import requests
from PIL import Image
import io
import json

image_file_path = "photos/human2.png"
text_prompt = "the object being presented"

url = "http://165.1.66.13:80/annotate"

with open(image_file_path, "rb") as image_file:
    response = requests.post(url, params={"text_prompt": text_prompt}, files={"photo": image_file})

if response.status_code == 200:

    # Convert result from bytes to list
    response_string = response.content.decode('utf-8')
    response_list = json.loads(response_string)
    print(response_list)

else:
    print(f"Request failed with status code {response.status_code}")
    print("Response content:", response.content.decode())