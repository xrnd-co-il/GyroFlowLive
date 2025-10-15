use crate::{ stabilization::*, zooming::* };
use exr::prelude::*;
use rayon::{ slice::ParallelSliceMut, iter::IndexedParallelIterator, iter::ParallelIterator };
use crate::StabilizationManager;

pub fn generate_stmaps(stab: &StabilizationManager, per_frame: bool) -> impl Iterator<Item = (String, usize, Vec<u8>, Vec<u8>)> { // (frame, undistort, redistort)

    //gets the with and height from the stabilization manager.
    let (width, height) = {
        let params = stab.params.read();
        (params.size.0, params.size.1)
    };


    //formates the filename base. 
    let filename_base = {
        let lens = stab.lens.read();
        format!("{}-{}-{}-{}", crate::filesystem::get_filename(&stab.input_file.read().url), lens.camera_brand, lens.camera_model, lens.lens_model)
            .replace("/", "-")
            .replace("\\", "-")
            .replace(":", "-")
            .replace("+", "-")
            .replace("'", "-")
            .replace("\"", "-")
            .replace(" ", "-")
    };

    /* critical section
    builds the compute params from the stabilization manager */
    let mut compute_params = ComputeParams::from_manager(&stab);
    compute_params.adaptive_zoom_window = -1.0; // static zoom
    if !per_frame {
        //if were not generating per frame then do a single template frame and siable rolling shutter (readout time 0).
        compute_params.frame_count = 1;
        compute_params.frame_readout_time = 0.0;
    }
    //changing params, need to get more info about this section.
    compute_params.keyframes.clear();
    compute_params.suppress_rotation = true;
    compute_params.fov_algorithm_margin = 0.0;
    compute_params.fovs.clear();
    compute_params.minimal_fovs.clear();

    //kernal flags for the gpu later
    let mut kernel_flags = KernelParamsFlags::empty();
    kernel_flags.set(KernelParamsFlags::HAS_DIGITAL_LENS, compute_params.digital_lens.is_some()); //if digital lens is present
    kernel_flags.set(KernelParamsFlags::HORIZONTAL_RS, compute_params.frame_readout_direction.is_horizontal()); //whether the readout is horizontal


    //iterator over the frames to generate the stmaps. 
    //frame params is the index of the frame
    (0..compute_params.frame_count).map(move |frame| {
        let timestamp = crate::timestamp_at_frame(frame as i32, compute_params.scaled_fps); //compute the timestamp for the frame


        //finding FoV/size by probing a grid of points around the edges of the frame and undistorting them.
        let org_output_size = (width, height); //original output size
        compute_params.fov_scale = 1.0;
        compute_params.width              = width; compute_params.height              = height;
        compute_params.output_width       = width; compute_params.output_height       = height;

        let mut transform = FrameTransform::at_timestamp(&compute_params, timestamp, frame); //compute the frame transform
        //set the kernel params for the transform
        transform.kernel_params.width  = width as i32;
        transform.kernel_params.height = height as i32;
        transform.kernel_params.output_width  = width as i32;
        transform.kernel_params.output_height = height as i32;
        transform.kernel_params.flags = kernel_flags.bits();

        //still need to be understood
        //convert mesh to f64 if donwstream expect double
        let mesh_data = transform.mesh_data.iter().map(|x| *x as f64).collect::<Vec<f64>>();

        let bbox = fov_iterative::FovIterative::new(&compute_params, org_output_size).points_around_rect(width as f32, height as f32, 31, 31); //`grid of points around the edges of the frame  
        let (camera_matrix, distortion_coeffs, _p, rotations, is, mesh) = FrameTransform::at_timestamp_for_points(&compute_params, &bbox, timestamp, Some(frame), false); //get the frame transform for the points
        let undistorted_bbox = undistort_points(&bbox, camera_matrix, &distortion_coeffs, rotations[0], None, Some(rotations), &compute_params, 1.0, timestamp, is, mesh); //undistort the points

        //find the bounding box of the undistorted points
        let mut min_x = 0.0;
        let mut min_y = 0.0;
        let mut max_x = 0.0;
        let mut max_y = 0.0;
        for (x, y) in undistorted_bbox {
            min_x = x.min(min_x);
            min_y = y.min(min_y);
            max_x = x.max(max_x);
            max_y = y.max(max_y);
        }
        let new_width  = (max_x - min_x).ceil() as usize;
        let new_height = (max_y - min_y).ceil() as usize;


        //update FoV
        compute_params.fov_scale = (new_width as f32 / width as f32).max(new_height as f32 / height as f32) as f64;
        compute_params.width              = new_width; compute_params.height              = new_height;
        compute_params.output_width       = new_width; compute_params.output_height       = new_height;


        //recompute frametransform with updated parameters and set kernal params
        transform = FrameTransform::at_timestamp(&compute_params, timestamp, frame);
        transform.kernel_params.width  = new_width as i32;
        transform.kernel_params.height = new_height as i32;
        transform.kernel_params.output_width  = new_width as i32;
        transform.kernel_params.output_height = new_height as i32;
        transform.kernel_params.flags = kernel_flags.bits();

        let r_limit_sq = transform.kernel_params.r_limit * transform.kernel_params.r_limit;

            //build undistordted map as EXR in parallel
            //calculate for each pixel (x,y ) the ssource pixel
            //EXR is a file form that comntatin indepth information about pixels and image.
            //we create a lookup table for pixels so we can rotate them
        let undist = parallel_exr(new_width, new_height, |x, y| {
            ///////////////////////////////////////////////////////////////////
            // Calculate source `y` for rolling shutter
            let mut sy = if compute_params.frame_readout_direction.is_horizontal() {
                (x.round() as i32).min(transform.kernel_params.width).max(0) as usize
            } else {
                (y.round() as i32).min(transform.kernel_params.height).max(0) as usize
            };
            if transform.kernel_params.matrix_count > 1 {
                let idx = transform.kernel_params.matrix_count as usize / 2;
                if let Some(pt) = Stabilization::rotate_and_distort((x as f32, y as f32), idx, &transform.kernel_params, &transform.matrices, &compute_params.distortion_model, compute_params.digital_lens.as_ref(), r_limit_sq, &mesh_data) {
                    if compute_params.frame_readout_direction.is_horizontal() {
                        sy = (pt.0.round() as i32).min(transform.kernel_params.width).max(0) as usize;
                    } else {
                        sy = (pt.1.round() as i32).min(transform.kernel_params.height).max(0) as usize;
                    }
                }
            }
            ///////////////////////////////////////////////////////////////////

            let idx = sy.min(transform.kernel_params.matrix_count as usize - 1);
            Stabilization::rotate_and_distort((x as f32, y as f32), idx, &transform.kernel_params, &transform.matrices, &compute_params.distortion_model, compute_params.digital_lens.as_ref(), r_limit_sq, &mesh_data)
        });

        //returning to the original size
        compute_params.width              = width; compute_params.height              = height;
        compute_params.output_width       = width; compute_params.output_height       = height;


        //build redistort map as EXR in parallel
        let dist = parallel_exr(width, height, |x, y| {
            let distorted = [(x as f32, y as f32)];
            let (camera_matrix, distortion_coeffs, _p, rotations, is, mesh) = FrameTransform::at_timestamp_for_points(&compute_params, &distorted, timestamp, Some(frame), true);
            undistort_points(&distorted, camera_matrix, &distortion_coeffs, rotations[0], None, Some(rotations), &compute_params, 1.0, timestamp, is, mesh).first().copied()
        });

        (filename_base.clone(), frame, dist, undist) //RETURN THis tuple per frame
    })
}
//the parallel exr function
fn parallel_exr(width: usize, height: usize, cb: impl Fn(f32, f32) -> Option<(f32, f32)> + Sync) -> Vec<u8> {
    let mut coords = vec![0.0f32; width * height * 2];
    coords.par_chunks_mut(width * 2).enumerate().for_each(|(y, row)| { // Parallel iterator over buffer rows
        row.chunks_mut(2).enumerate().for_each(|(x, pix)| { // iterator over row pixels
            if let Some(pt) = cb(x as f32, y as f32) {
                pix[0] = pt.0;
                pix[1] = pt.1;
            }
        });
    });
    let channels = SpecificChannels::rgb(|Vec2(x, y)| (
                   coords[y * width * 2 + x * 2 + 0] / width as f32,
            1.0 - (coords[y * width * 2 + x * 2 + 1] / height as f32),
            0.0
    ) );
    let mut data = Vec::new();
    let mut img = Image::from_channels((width, height), channels);
    img.layer_data.encoding.compression = Compression::ZIP16;
    if let Err(e) = img.write().to_buffered(std::io::Cursor::new(&mut data)) {
        ::log::error!("Failed to write EXR: {e:?}");
    }
    data
}
