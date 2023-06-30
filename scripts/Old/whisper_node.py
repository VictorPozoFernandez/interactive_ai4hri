import rospy
from std_msgs.msg import String
import speech_recognition as sr
import queue
import threading
import openai
import os
import tempfile


def main():

    global audio_control
    audio_control = "RESUME"

    # Initialize the whisper ROS node and a publisher for the AI4HRI utterance topic
    rospy.init_node("whisper", anonymous=True)
    pub = rospy.Publisher('/ai4hri/utterance', String, queue_size= 1) 
    rospy.Subscriber("/ai4hri/audio_control", String, callback)

    # Put the desired configuration 
    energy = 1000
    pause = 0.5

    # Print available microhpones PC
    for index, name in enumerate(sr.Microphone.list_microphone_names()): print(f'{index}, {name}')

    audio_queue = queue.Queue()
    result_queue = queue.Queue()
    rate = rospy.Rate(1)

    # Start different threads to record and transcribe the audio
    threading.Thread(target=record_audio,
                    args=(audio_queue, energy, pause, rate)).start()

    threading.Thread(target=transcribe_audio,
                    args=(audio_queue, result_queue, rate)).start()

    # Main loop for the ROS node
    while not rospy.is_shutdown():

        # Get the transcribed utterance from the result_queue
        utterance = String()
        utterance = result_queue.get() 
        
        # Publish the utterance if it's longer than 10 characters
        if (len(utterance) > 12) and ("Amara.org" not in utterance) and ("for watching" not in utterance):
            utterance = utterance.replace(",", "")
            utterance = utterance.replace("'", "")
            pub.publish(utterance)
        
        rate.sleep()


def callback(msg):

    global audio_control
    audio_control = msg.data


def record_audio(audio_queue, energy, pause, rate):

    global audio_control

    # Initialize the recognizer and set energy and pause thresholds
    r = sr.Recognizer()
    r.energy_threshold = energy
    r.pause_threshold = pause

    # Open the microphone with the specified sample rate and device index
    counter = 0
    found_micro = False
    while (counter < 15) and (found_micro == False) :
        try:      
            with sr.Microphone(sample_rate=16000,  device_index=counter) as source:

                # Clear the console
                for x in range(5):
                    print("")
                rospy.loginfo("Node whisper initialized. Listening...")

                # Record audio while the ROS node is running
                while not rospy.is_shutdown():
                    
                    audio = r.listen(source)

                    if audio_control == "RESUME":                   
                        audio_queue.put_nowait(audio)

                    found_micro = True
        except:
            counter += 1
            if counter >= 15:
                print("Microphone not found")
                break

def transcribe_audio(audio_queue, result_queue, rate):

    openai.organization = os.environ.get("OPENAI_ORG_ID")
    openai.api_key = os.environ.get("OPENAI_API_KEY")
 
    # Transcribe the audio while the ROS node is running
    while not rospy.is_shutdown():

        # Get the audio data from the audio_queue and transcribe it using the whisper audio model
        audio_data = audio_queue.get()
        
        # Save the audio_data as a temporary file
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_file.write(audio_data.get_wav_data())
            temp_file_name = temp_file.name

        language = os.environ.get("LANGUAGE_WHISPER")
        if language == None:
            language = "en" #Change language (english = "en", spanish = "es", french = "fr", german = "de", italian = "it", japanese = "ja")  

        # Transcribe the temporary audio file
        with open(temp_file_name, "rb") as audio_file:
            result = openai.Audio.transcribe("whisper-1", audio_file, language=language)          
            
        # Remove the temporary file
        os.remove(temp_file_name)

        predicted_text = result["text"]
        
        # Add the predicted text to the result_queue
        result_queue.put_nowait(predicted_text)
        rate.sleep()


if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
    
