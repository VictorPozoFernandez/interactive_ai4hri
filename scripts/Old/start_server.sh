sudo apt update
sudo apt upgrade -y
sudo apt install nginx -y
sudo apt install git -y

pip install pybind11
pybind11_path=$(python -c "import pybind11; print(pybind11.get_include())")
export C_INCLUDE_PATH=$C_INCLUDE_PATH:$pybind11_path
export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:$pybind11_path
git clone https://github.com/IDEA-Research/GroundingDINO.git
cd GroundingDINO/
pip3 install -q -e .
mkdir weights
cd weights
wget -q https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth

cd ..
cd ..
pip install git+https://github.com/facebookresearch/segment-anything.git
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth

pip install fastapi
pip install uvicorn
pip install python-multipart
pip install pinecone-client
pip install transformers
pip install accelerate
pip install pillow
pip install requests



git clone https://github.com/VictorPozoFernandez/starting_server.git

# Get public IP using an external API
public_ip=$(curl -s https://api.ipify.org)

# Check if the IP address was fetched successfully
if [[ -z "$public_ip" ]]; then
    echo "Error: Could not fetch public IP address"
    exit 1
fi

# Set the environment variable with the public IP
export PUBLIC_IP="$public_ip"

# Read the fastapi_nginx file
cd starting_server
template=$(cat fastapi_nginx_template)

# Replace the placeholder with the value of the PUBLIC_IP environment variable
output="${template//\{PUBLIC_IP\}/$PUBLIC_IP}"

# Save the result to a new file
echo "$output" > fastapi_nginx

sudo cp fastapi_nginx /etc/nginx/sites-enabled/
sudo service nginx restart

cd ..
mv starting_server/main.py ~/
python3 main.py

