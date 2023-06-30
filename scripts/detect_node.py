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

    # Initialize the detector ROS node and subscribe to utterance topic
    rospy.init_node("detector", anonymous=True)
    rospy.Subscriber("/ai4hri/utterance", String, callback)
    rospy.loginfo("Node detect_node initialized. Listening...")

    while not rospy.is_shutdown():

        if flag != None:

            # Initialize publisher for the utterance_to_agent topic
            pub = rospy.Publisher('/ai4hri/utterance_to_agent', String, queue_size= 1) 

            # Get new utterance from message and classify it
            utterance = flag
            print("")
            print("----------------------------------------------")
            print("Customer: " + utterance)

            rospy.sleep(1)
            elements = extraction_list_products()            

            with concurrent.futures.ThreadPoolExecutor() as executor:

                future_1 = executor.submit(classify_sentence, utterance, sentence_classification)
                future_2 = executor.submit(classify_sentence, utterance, sentence_classification_2)
                future_3 = executor.submit(reasoning_sentence, elements, utterance, model_reasoning)
                #Classificar si un modelo ha sido mencionado. En caso negativo, saltarse proceso de identificar modelo y pasar directamente a buscar la info en la sql
                
                # Retrieve results from futures.
                classification_result = future_1.result()
                classification_result_2 = future_2.result()
                identified_models = future_3.result()

            print("Sentence classification: (" + str(classification_result["Detection"]) + ")" + " (" + str(classification_result_2["Detection"]) + ")")

            if classification_result["Detection"] == "James" and classification_result_2["Detection"] == "Known":

                if (identified_models["Output"] == "Lack Information"):
                    
                    identified_models = identified_models["Detection"]
                    question = question_reasoning(identified_models)

                    if question != "NULL":
                        print("")
                        print("----------------------------------------------")
                        print("Shopkeeper: " + question)

                        answer = ask_question(question)
                        print("")
                        print("----------------------------------------------")
                        print("Customer: " + answer)

                        identified_models = model_reasoning(elements, "Question: " + question + "Answer: " + answer)


                # Send the utterance to the agent
                if (identified_models["Output"] != "Lack Information"):
                    pub.publish(utterance + ": " + str(identified_models["Output"]) + "(Product_ID, 'Model')")

                else:
                    response = "I'm sorry, but I couldn't identify the device's model you are referring to."
                    print("")
                    print("----------------------------------------------")
                    print("James: " + str(response))
                    answer_customer(response)

            elif classification_result["Detection"] == "James":
                pub.publish(utterance)

            flag = None

        rospy.sleep(1)

def callback(msg):

    global flag
    flag = msg.data

    
def classify_sentence(utterance, classifier_function):
    classification_result = classifier_function(utterance)
    return classification_result

def reasoning_sentence(elements, utterance, classifier_function):
    classification_result = classifier_function(elements, utterance)
    return classification_result


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

     Customer: Is the Sony Alpha A6000 cheaper than the Nikon Coolpix?
    List: [(1, 'Nikon Coolpix S2800'), (2, 'Sony Alpha a6000'), (3, 'Canon EOS 5D Mark III'), (4, 'Xiaomi T11 Pro'), (5, 'Huawei airpods 4j'), (6, 'Sony Alpha a5000'), (7, 'Xiaomi Mi A3')]
    You: {"Reasoning": "The Customer is comparing two elements ((1, 'Nikon Coolpix S2800'), (2, 'Sony Alpha a6000'))", "Detection" : "(1, 'Nikon Coolpix S2800'), (2, 'Sony Alpha a6000')", "Output": "(1, 'Nikon Coolpix S2800'), (2, 'Sony Alpha a6000')"}
    
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
