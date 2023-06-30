import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import openai
import os
import queue
import tempfile

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

    rospy.sleep(1)
    rospy.spin()


def callback(msg):
    audio_data = msg.data

    # Save the audio_data as a temporary file
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
        temp_file.write(audio_data)
        temp_file_name = temp_file.name

    language = os.environ.get("LANGUAGE_WHISPER")
    if language == None:
        language = "en" #Change language (english = "en", spanish = "es", french = "fr", german = "de", italian = "it", japanese = "ja")  

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


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
