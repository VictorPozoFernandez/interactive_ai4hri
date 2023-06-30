import pinecone
import numpy as np
import requests
import os
import json


def query_embeddings():

    pinecone.init(api_key="04d65f27-278f-40f0-9cfb-c907b84115a7", environment="us-east4-gcp")
    index = pinecone.Index("text-embeddings1")

    query_response = index.query(
        namespace="example-namespace",
        top_k=10,
        include_metadata=True,
        vector=np.random.rand(768).tolist())

    print(query_response.matches)
    

def save_embedding(image_path, ProductID, element):

    # Open the image file
    with open(image_path, "rb") as image_file:

        url = "http://" + str(os.environ.get("CLOUD_IP")) + ":80/save_embedding"
        files = {"photo": image_file}

        # Send a POST request to the specified URL with the image file as the payload.
        response = requests.post(url, params={"text_prompt":ProductID, "text_prompt_2":element}, files=files)

        # Check if the response status code is 200, indicating a successful request.
        if response.status_code == 200:
            # Convert result from bytes to list
            response_string = response.content.decode('utf-8')
            response_list = json.loads(response_string)
            return response_list
        
        # If the response status code is not 200, handle the error.
        else:
            # Return an error message with the status code.
            return f"Error: {response.status_code}"
 
        
image_path = "/home/victor/catkin_ws/src/visual_ai4hri/photos/canon_powershot.png"
result = save_embedding(image_path, "9", "Canon PowerShot SX740HS")
print(result)

query_embeddings()