import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import openai
import os
import queue
import tempfile
from dynamic_reconfigure.server import Server
from interactive_ai4hri.cfg import audiotranscriber_ai4hriConfig

openai.organization = os.environ.get("OPENAI_ORG_ID")
openai.api_key = os.environ.get("OPENAI_API_KEY")


def main():
    # Initialize the whisper ROS node and a publisher for the AI4HRIutterance topic
    rospy.init_node("audio_transcriber", anonymous=True)
    rospy.Subscriber("/ai4hri/audio_data", AudioData, callback)
    rospy.loginfo("Node audio_transcriber initialized. Listening...")

    #The system never listens to the first utterance. This snippet code publishes a first utterance that will be ignored.
    pub = rospy.Publisher('/ai4hri/utterance', String, queue_size= 1)
    pub.publish("------")
    srv = Server(audiotranscriber_ai4hriConfig, callback2)

    rospy.sleep(1)
    rospy.spin()


def callback(msg):
    global language
    audio_data = msg.data

    # Save the audio_data as a temporary file
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
        temp_file.write(audio_data)
        temp_file_name = temp_file.name

    if language == None:
        language = "en" #Change language (english = "en", spanish = "es", french = "fr", german = "de", italian = "it", japanese = "ja")  

    print(language)
    
    # Transcribe the audio file
    with open(temp_file_name, "rb") as audio_file:
        result = openai.Audio.transcribe("whisper-1", audio_file, language=language)          

    utterance = result["text"]
    
    # Publish the utterance if it's longer than 10 characters
    if (len(utterance) > 12) and ("Amara.org" not in utterance) and ("for watching" not in utterance) and ("Thank" not in utterance):
        utterance = utterance.replace(",", "")
        utterance = utterance.replace("'", "")
        pub = rospy.Publisher('/ai4hri/utterance', String, queue_size= 1)
        pub.publish(utterance)


def callback2(config, level):

    rospy.loginfo("""Reconfigure Request: {language_str_param} """.format(**config))
    
    global language
    language=config.language_str_param

    return config

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
