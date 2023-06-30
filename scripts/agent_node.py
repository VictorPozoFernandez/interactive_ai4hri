import os
from langchain.chat_models import ChatOpenAI
from langchain.agents import initialize_agent
from langchain.tools import BaseTool
from langchain import SQLDatabase
from langchain.chains import SQLDatabaseChain
from std_msgs.msg import String
import os
import rospy


def main():

    # Initialize the agent ROS node and subscribe to utterance topic
    rospy.init_node("agent", anonymous=True)
    rospy.loginfo("Node agent initialized. Listening...")
    rospy.Subscriber("/ai4hri/utterance_to_agent", String, callback)
    
    rospy.spin()


def callback(msg):

    # Get new utterance from message and classify it
    utterance = msg.data
    
    try:
        result = activate_agent(utterance)
        
    except Exception as e:
        e = str(e)
        if not e.startswith("Could not parse LLM output: `"):
            raise e
        
        prefix = "Could not parse LLM output: `"
        suffix = "`"

        if e.startswith(prefix):
            e = e[len(prefix):]
        if e.endswith(suffix):
            e = e[:-len(suffix)]

        result = e   

    print("")
    print("----------------------------------------------")
    print("James: " + result)

    #Answer the question
    pub3 = rospy.Publisher('/ai4hri/response_robot', String, queue_size= 1)
    rospy.sleep(1)
    pub3.publish(result)
    rospy.sleep(1)


def activate_agent(utterance):
    llm = ChatOpenAI(model_name="gpt-3.5-turbo-0613", temperature=0.0, openai_api_key=os.environ.get("OPENAI_API_KEY"))
    tools = [SearchTool()]

    agent = initialize_agent(
        agent='zero-shot-react-description', 
        tools=tools, 
        llm=llm,
        verbose=True,
        max_iterations=1,
        early_stopping_method="generate")


    result = agent(utterance)

    return result["output"]


class SearchTool(BaseTool):
    name = "Search"
    description = "Use this tool to search information of a product"

    def _run(self, search: str):

        script_dir = os.path.dirname(os.path.abspath(__file__))  # Returns the absolute path to the script
        parent_dir = os.path.dirname(script_dir)
        file_path = os.path.join(parent_dir, 'Camera_Store.db')

        db = SQLDatabase.from_uri("sqlite:////" + file_path)
        llm = ChatOpenAI(model_name="gpt-3.5-turbo-0613", temperature=0.0, openai_api_key=os.environ.get("OPENAI_API_KEY"))

        db_chain = SQLDatabaseChain(llm=llm, database=db, verbose=True, return_intermediate_steps=True)
        db_result = db_chain(search)

        return db_result["intermediate_steps"][-1]
    

    def _arun(self, query:str):
        raise NotImplementedError("This tool does not support async")


if __name__ == '__main__':

    try:
        main()
    
    except rospy.ROSInterruptException:
        pass






