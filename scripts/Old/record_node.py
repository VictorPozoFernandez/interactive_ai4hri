import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import speech_recognition as sr
import numpy as np

global audio_control
audio_control = "RESUME"


def callback(msg):
    global audio_control
    audio_control = msg.data


def record_audio(pub):

    global audio_control

    # Initialize the recognizer and set energy and pause thresholds
    r = sr.Recognizer()
    energy_threshold = 220

    # Open the microphone with the specified sample rate and device index
       
    try:  

        # Print available microphones PC

        rospy.sleep(2)
        print("")
        for index, name in enumerate(sr.Microphone.list_microphone_names()): print(f'{index}, {name}')
        print('')
        counter = input('Choose a microphone device:')

        with sr.Microphone(sample_rate=16000,  device_index=int(counter)) as source:

            # Clear the console
            for x in range(4):
                print("")
            rospy.loginfo("Node record_node initialized. Listening...")

            complete_audio = None
            counter = 0

            # Record audio while the ROS node is running
            while not rospy.is_shutdown():
                
                if (audio_control == "RESUME"):
                    audio = r.record(source, duration=1)  # Record for a small duration
                   
                    # Check for silence (low energy level)
                    energy_level = get_audio_energy(audio)

                    if energy_level > energy_threshold:

                         # Concatenate audio chunks if any speech is detected
                        if complete_audio == None:
                            complete_audio = sr.AudioData(previous_audio.frame_data + audio.frame_data, audio.sample_rate, audio.sample_width)
                        else:
                            complete_audio = sr.AudioData(complete_audio.frame_data + audio.frame_data, audio.sample_rate, audio.sample_width)
                    
                    elif (energy_level < energy_threshold) and (complete_audio != None) and (counter < 2):
                        
                        complete_audio = sr.AudioData(complete_audio.frame_data + audio.frame_data, audio.sample_rate, audio.sample_width)
                        counter = counter + 1

                    elif (energy_level < energy_threshold) and (complete_audio != None):  
                       
                        complete_audio = sr.AudioData(complete_audio.frame_data + audio.frame_data, audio.sample_rate, audio.sample_width)
                        counter = 0
                        wav_data = complete_audio.get_wav_data()
                        pub.publish(wav_data)
                        complete_audio = None

                    previous_audio = audio
                    

                else:
                    rospy.sleep(0.1)
                
        
    except Exception as e: 
            print(e)
            print("Microphone not found")


def get_audio_energy(audio):
    """Calculate the energy of an AudioData object"""
    # Convert audio data to numpy array
    audio_data = np.frombuffer(audio.frame_data, np.int16)
    # Calculate RMS value
    rms_val = np.sqrt(np.mean(audio_data**2))
    return 10000/rms_val


def main():
    # Initialize the whisper ROS node and a publisher for the AI4HRI utterance topic
    rospy.init_node("audio_recorder", anonymous=True)
    pub = rospy.Publisher('/ai4hri/audio_data', AudioData, queue_size= 1)
    rospy.Subscriber("/ai4hri/audio_control", String, callback)

    record_audio(pub)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass