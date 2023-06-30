sudo apt update
sudo apt upgrade -y
sudo apt install nginx -y
sudo apt install git -y
pip install fastapi
pip install pillow
pip install requests
pip install transformers
pip install uvicorn
pip install accelerate
pip install python-multipart

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
python3 main.py
