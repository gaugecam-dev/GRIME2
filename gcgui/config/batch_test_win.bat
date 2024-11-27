REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~ GENERAL ~~~~~~~~~~~~~~~~~~~~~~~~~~~
grime2cli --version
grime2cli --help
grime2cli --show_metadata --source "c:/gaugecam/config/2022_demo/20220715_KOLA_GaugeCam_001.JPG"
grime2cli --make_gif --source "c:/gaugecam/config/2012_demo/06/" --result_image "c:/gaugecam/results/demo.gif" --scale 0.20 --delay_ms 1000

REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~ BOW-TIE ~~~~~~~~~~~~~~~~~~~~~~~~~~~
grime2cli --create_calib bowtie --source "c:/gaugecam/config/2012_demo/05/NRmarshDN-12-05-31-23-45.jpg" --calib_json "c:/gaugecam/config/calib.json" --csv_file "c:/gaugecam/config/calibration_target_world_coordinates.csv" --result_image "" --waterline_roi 474 84 591 84 483 580 604 582 --calib_roi 317 29 436 555
grime2cli --find_line --timestamp_from_filename --timestamp_start_pos 10 --timestamp_format "yy-mm-dd-HH-MM" --source "c:/gaugecam/config/2012_demo/06/NRmarshDN-12-06-30-10-30.jpg" --calib_json "c:/gaugecam/config/calib.json" --result_image "c:/gaugecam/results/find_line_result.png"
grime2cli --run_folder --timestamp_from_filename --timestamp_start_pos 10 --timestamp_format "yy-mm-dd-HH-MM" --source "c:/gaugecam/config/2012_demo/06/" --calib_json "c:/gaugecam/config/calib.json" --csv_file "c:/gaugecam/results/folder.csv" --result_folder "c:/gaugecam/results/"

REM ~~~~~~~~~~~~~~~~~~~~~~~~~~~ STOP SIGN ~~~~~~~~~~~~~~~~~~~~~~~~~~~
grime2cli --create_calib octagon --source "c:/gaugecam/config/2022_demo/20220715_KOLA_GaugeCam_001.JPG/20220715_KOLA_GaugeCam_001.JPG" --calib_json "c:/gaugecam/config/calib_stopsign.json" --result_image "c:/gaugecam/results/calib_result_stopsign.png" --zero_offset 4.000000 --facet_length 0.599000 --waterline_roi 1078 776 1266 783 1071 1054 1244 1068 --calib_roi 971 454 354 345
grime2cli --find_line --timestamp_from_exif --timestamp_start_pos 0 --timestamp_format "yyyy-mm-dd-HH-MM" --source "c:/gaugecam/config/2022_demo/20220715_KOLA_GaugeCam_001.JPG" --calib_json "c:/gaugecam/config/calib_stopsign.json" --result_image "c:/gaugecam/results/find_line_result_stopsign.png"
grime2cli --run_folder --timestamp_from_exif --timestamp_start_pos 0 --timestamp_format "yyyy-mm-dd-HH-MM" --source "c:/gaugecam/config/2022_demo/" --calib_json "c:/gaugecam/config/calib_stopsign.json" --csv_file "c:/gaugecam/results/folder_stopsign.csv" --result_folder "c:/gaugecam/results/"

