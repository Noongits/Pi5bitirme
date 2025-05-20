import variables
import detect_and_crop
import angle_classification
import stereo_distance

def localise_eiffel():
    detect_and_crop.process_frames(variables.leftcam, variables.rightcam)
    angle_classification.main()
    stereo_distance.main()

    # TODO: eiffel angle and distance are saved into variables.py. use them to self-localise
