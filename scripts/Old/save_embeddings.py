import os
import requests
import argparse
import re

def save_embedding(image_path):

    filename = os.path.basename(image_path)
    ProductID, element = filename.split('-', 1)
    element = element.rsplit('.', 1)[0]
    element = re.sub(r' \(\d+\)$', '', element)

    with open(image_path, "rb") as image_file:
        url = "http://" + str(os.environ.get("CLOUD_IP")) + ":80/save_embedding"
        files = {"photo": image_file}

        response = requests.post(url, params={"text_prompt":ProductID, "text_prompt_2":element}, files=files)

        if response.status_code == 200:
            response_string = response.content.decode('utf-8')
            return response_string
        else:
            return f"Error: {response.status_code}"


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Embedding script")
    parser.add_argument("--dir_path", required=True, help="Path to the directory containing images")
    
    args = parser.parse_args()

    # Loop over all files in the directory
    for filename in os.listdir(args.dir_path):
        if filename.endswith((".png", ".jpg", ".jpeg")):  # add more image extensions if needed
            image_path = os.path.join(args.dir_path, filename)
            result = save_embedding(image_path)
        
        while result == "Error: 504" or result == "Error: 500":
            result = save_embedding(image_path)

        print(result)

