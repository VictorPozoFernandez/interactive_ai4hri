import rospy
from std_msgs.msg import String
import os
from langchain.chat_models import ChatOpenAI
from langchain.schema import HumanMessage, SystemMessage
from langchain.prompts import PromptTemplate
from cv_bridge import CvBridge
import cv2
import requests
import json
import sqlite3
import re
from sensor_msgs.msg import Image
import concurrent.futures
import ast

bridge = CvBridge()

def main():

    global flag
    flag = None
    global PEPPER

    # Initialize the detector ROS node and subscribe to utterance topic
    rospy.init_node("detector", anonymous=True)


    try:
        rospy.wait_for_message("/naoqi_driver_node/camera/front/image_raw", Image, timeout=2)
        PEPPER = True
        rospy.loginfo("Node detector initialized in PEPPER mode. Listening...")
    
    except:
        PEPPER = False
        rospy.loginfo("Node detector initialized. Listening...")


    rospy.Subscriber("/ai4hri/utterance", String, callback)

    while not rospy.is_shutdown():

        if flag != None:

            # Initialize publisher for the utterance_to_agent topic
            pub = rospy.Publisher('/ai4hri/utterance_to_agent', String, queue_size= 1) 
            pub2 = rospy.Publisher('/ai4hri/take_photo', String, queue_size= 1)

            # Get new utterance from message and classify it
            utterance = flag
            print("")
            print("----------------------------------------------")
            print("Customer: " + utterance)

            rospy.sleep(1)
            
            if PEPPER == True:
                pass
            else:
                pub2.publish("Take photo")

            with concurrent.futures.ThreadPoolExecutor() as executor:

                future_1 = executor.submit(classify_sentence, utterance, sentence_classification)
                future_2 = executor.submit(classify_sentence, utterance, sentence_classification_2)
                future_3 = executor.submit(take_photo)
                
                # Retrieve results from futures.
                classification_result = future_1.result()
                classification_result_2 = future_2.result()
                cv_image = future_3.result()

            print("Sentence classification: (" + str(classification_result["Detection"]) + ")" + " (" + str(classification_result_2["Detection"]) + ")")

            if classification_result["Detection"] == "James":
                   
                while True:
                    
                    elements = element_identification(cv_image)
                    print(elements)

                    if isinstance(elements, list):
                        break

                    elif classification_result_2["Detection"] == "Known":
                        elements = extraction_list_products()
                        break
                    
                    else:
                        question = "Please bring the element closer to the camera for better identification."
                        print("")
                        print("----------------------------------------------")
                        print("James: " + question)
                        
                        answer = ask_question(question)
                        print("")
                        print("----------------------------------------------")
                        print("Customer: " + answer)

                        if PEPPER == True:
                            image = rospy.wait_for_message("/naoqi_driver_node/camera/front/image_raw", Image) #Topic where Pepper publishes images of its camera.
                        
                        else:
                            pub2.publish("Take photo")
                            image = rospy.wait_for_message("/ai4hri/image", Image)
 
                        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
                        
                    
                    rospy.sleep(1)
                
                if len(elements) == 1:
                    identified_model = {"Output" : elements[0]}

                else:
                    identified_model = model_reasoning(elements, utterance)
                    print(identified_model)

                if (identified_model["Output"] == "Lack Information"):
                    
                    identified_models = identified_model["Detection"]
                    question = question_reasoning(identified_models)

                    if question != "NULL":
                        print("")
                        print("----------------------------------------------")
                        print("Shopkeeper: " + question)

                        answer = ask_question(question)
                        print("")
                        print("----------------------------------------------")
                        print("Customer: " + answer)

                        identified_model = model_reasoning(elements, "Question: " + question + "Answer: " + answer)


                # Send the utterance to the agent
                if (identified_model["Output"] != "Lack Information"):
                    pub.publish(utterance + ": " + str(identified_model["Output"]) + "(Product_ID, 'Model')")

                else:
                    response = "I'm sorry, but I couldn't identify the device's model you are referring to."
                    print("")
                    print("----------------------------------------------")
                    print("James: " + str(response))
                    answer_customer(response)


            flag = None

        rospy.sleep(1)

def callback(msg):

    global flag
    flag = msg.data

    
def classify_sentence(utterance, classifier_function):
    classification_result = classifier_function(utterance)
    return classification_result


def take_photo():
    global PEPPER

    try:
        
        # Wait for a single message from the topic

        if PEPPER == True:
            image = rospy.wait_for_message("/naoqi_driver_node/camera/front/image_raw", Image) #Topic where Pepper publishes images of its camera. 

        else:
            image = rospy.wait_for_message("/ai4hri/image", Image) 
        
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
    
    except rospy.ROSException as e:
        print("Timeout waiting for message from topic /ai4hri/image")
    
    return cv_image


def sentence_classification(utterance):

    # Set OpenAI API credentials
    openai_api_key = os.environ.get("OPENAI_API_KEY")

    # Prepare prompt to send, using JSON format
    chat = ChatOpenAI(model_name="gpt-3.5-turbo-0613", temperature=0, openai_api_key=openai_api_key)


    system_prompt = """
    You are a helpful assistant called "James". If the Input contains a question directly to you, output "James". Otherwise, output "null".
    Beware that the name "James" may be misspelled during the transcription form audio to text, but even then you still have to identify that the question is directed to you. Examples of misspelling: "gems", "jaims". 
    
    Here there are some examples that illustrates how can you output your answer. The interactions appear in cronological order:

    Input: What's the price of this device james?
    You: {"Detection": "James"}

    Input: What's the price of this device?
    You: {"Detection": "null"}

    Output the answer only in JSON format.
    """

    user_template = """
    Input: {statement}
    """

    user_prompt_template = PromptTemplate(input_variables=["statement"], template=user_template)
    user_prompt = user_prompt_template.format(statement = utterance)

    prompt_history = [
        SystemMessage(content=system_prompt),
        HumanMessage(content=user_prompt)
    ]

    result = chat(prompt_history)
    data = extract_json(result.content)

    return data


def sentence_classification_2(utterance):

    # Set OpenAI API credentials
    openai_api_key = os.environ.get("OPENAI_API_KEY")

    # Prepare prompt to send, using JSON format
    chat = ChatOpenAI(model_name="gpt-3.5-turbo-0613", temperature=0, openai_api_key=openai_api_key)


    system_prompt = """
    You are a helpful assistant called "James". You are required to discern whether the Input specifically refers to all the model's names of the devices in question. 
    If all the model's names are explicitly mentioned, your output should be "Known". If a model's name is not stated, the output should be "Unknown".
    
    Here there are some examples that illustrates how can you output your answer. The interactions appear in cronological order:

    Input: What's the price of this device james?
    You: {"Reasoning": "The device's name is not mentioned.", "Detection": "Unknown"}

    Input: What's the price of the Sony Alpha?
    You: {"Reasoning": "The device's name is mentioned (Sony Apha)", "Detection": "Known"}

    Input: Could you tell me the difference between this camera and the Sony Alpha?
    You: {"Reasoning": Two elements are being compared. One of the device's names is mentioned (Sony Alpha), but the other is not ("this camera" model is Unknown)", "Detection": "Unknown"}

    Output the answer only in JSON format.
    """

    user_template = """
    Input: {statement}
    """

    user_prompt_template = PromptTemplate(input_variables=["statement"], template=user_template)
    user_prompt = user_prompt_template.format(statement = utterance)

    prompt_history = [
        SystemMessage(content=system_prompt),
        HumanMessage(content=user_prompt)
    ]

    result = chat(prompt_history)

    data = extract_json(result.content)

    return data


def extract_json(s):
    json_match = re.search(r'\{.*\}', s, re.DOTALL)
    if json_match:
        try:
            data = json.loads(json_match.group())
            return data
        except json.JSONDecodeError:
            print("Invalid JSON")
            return None
    else:
        print("No JSON found in the string")
        return {"Detection": "null", "Model": "null", "Output" : "null",}
    

def element_identification(image):

    # Save the frame as a .png file
    cv2.imwrite('photo.png', image)

    # Open the image file specified by 'image_path' in binary read mode.
    with open("photo.png", "rb") as image_file:

        url = "http://" + str(os.environ.get("CLOUD_IP")) + ":80/annotate"
        files = {"photo": image_file}
        text_prompt = "the object being presented"

        # Send a POST request to the specified URL with the image file as the payload.
        response = requests.post(url, params={"text_prompt": text_prompt}, files=files)

        # Check if the response status code is 200, indicating a successful request.
        if response.status_code == 200:
            # Convert result from bytes to list
            response_string = response.content.decode('utf-8')
            response_list = json.loads(response_string)
            return response_list
        
        # If the response status code is not 200, handle the error.
        else:
            # Return an error message with the status code.
            return f"Element not identified"


def model_reasoning(elements, statement = ""):

    # Set OpenAI API credentials
    openai_api_key = os.environ.get("OPENAI_API_KEY")
    chat = ChatOpenAI(model_name="gpt-3.5-turbo-0613", temperature=0, openai_api_key=openai_api_key)

    system_template = """
    You are a helpful assistant called "James" that helps to detect which device model and Product_ID is the customer talking about using the List as reference. If two or more models are detected, output "Lack Information". Be concise, don't give explainations.
    
    Here there are some examples that illustrates how can you output your answer. The interactions appear in cronological order:

    Customer: How much does this Sony Alpha camera cost?
    List: [(1, 'Nikon Coolpix S2800'), (2, 'Sony Alpha a6000'), (3, 'Canon EOS 5D Mark III'), (4, 'Xiaomi T11 Pro'), (5, 'Huawei airpods 4j'), (6, 'Sony Alpha a5000'), (7, 'Xiaomi Mi A3')]
    You: {"Reasoning": "The Customer is talking about a Sony Alpha model. There are two possible models that fit this criteria ((2, 'Sony Alpha a6000'), (6, 'Sony Alpha a5000'))", "Detection" : "(2, 'Sony Alpha a6000'), (6, 'Sony Alpha a5000')", "Output": "Lack Information"}

    Customer: How much does this Nikon Coolpix camera cost?
    List: [(1, 'Nikon Coolpix S2800'), (2, 'Sony Alpha a6000'), (3, 'Canon EOS 5D Mark III'), (4, 'Xiaomi T11 Pro'), (5, 'Huawei airpods 4j'), (6, 'Sony Alpha a5000'), (7, 'Xiaomi Mi A3')]
    You: {"Reasoning": "The Customer is talking about a Sony Alpha model. There is only one model that fit this criteria ((1, 'Nikon Coolpix S2800'))", "Detection" : "(1, 'Nikon Coolpix S2800')", "Output": "(1, 'Nikon Coolpix S2800')"}

    Customer: How much does this device cost?
    List: [(1, 'Nikon Coolpix S2800'), (2, 'Sony Alpha a6000'), (3, 'Canon EOS 5D Mark III')]
    You: {"Reasoning": "The Customer is not explicitely mentioning any model. All models fit this criteria ((1, 'Nikon Coolpix S2800'), (2, 'Sony Alpha a6000'), (3, 'Canon EOS 5D Mark III')", "Detection" : "(1, 'Nikon Coolpix S2800'), (2, 'Sony Alpha a6000'), (3, 'Canon EOS 5D Mark III')", "Output": "Lack Information"}

    Output the answer only in JSON format.
    """

    user_template = """
    Customer: {statement}
    List: {elements}
    """

    user_prompt_template = PromptTemplate(input_variables=["elements", "statement"], template=user_template)
    user_prompt = user_prompt_template.format( elements = elements, statement = statement)

    prompt_history = [
        SystemMessage(content=system_template),
        HumanMessage(content=user_prompt)
    ]

    result = chat(prompt_history)
    data = extract_json(result.content)

    return data
 
def question_reasoning(identified_models):
    
    try:
        identified_models = ast.literal_eval(identified_models)
        question = "Are you referring"

        for i, model in enumerate(identified_models):
            
            if i ==len(identified_models) - 1:
                question = question + " or the " + str(model[1]) + "?"
            else:
                question = question + " to the " + str(model[1]) + ","
        
        return question

    except Exception as e:
        print(e)
        return "NULL"





def extraction_list_products():

    # Connect to the Camera_Store database. Initialize the cursor for querying the database.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(script_dir)
    db_path = os.path.join(parent_dir, "Camera_Store.db")
    db = sqlite3.connect(db_path)
    
    mycursorGPT = db.cursor()

    # List to store extracted characteristics for each product
    list_models = []

    mycursorGPT.execute("SELECT Product_ID, Model FROM Camera;")
    models = mycursorGPT.fetchall()

    for model in models:
        list_models.append(model)

    # Return the list 
    return list_models


def ask_question(question):

    #Ask the question
    pub2 = rospy.Publisher('/ai4hri/audio_control', String, queue_size= 1) 
    pub3 = rospy.Publisher('/ai4hri/response_robot', String, queue_size= 1)
    rospy.sleep(1)
    pub3.publish(question)
    rospy.sleep(1)

    try:
        
        # Wait for a single message from the topic
        msg = rospy.wait_for_message("/ai4hri/utterance", String)
        return msg.data
    
    except rospy.ROSException as e:
        print("Timeout waiting for message from topic /ai4hri/utterance")
        return ""

def answer_customer(question):

    #Ask the question
    pub2 = rospy.Publisher('/ai4hri/audio_control', String, queue_size= 1) 
    pub3 = rospy.Publisher('/ai4hri/response_robot', String, queue_size= 1)
    rospy.sleep(1)
    pub3.publish(question)
    rospy.sleep(1)



if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass
