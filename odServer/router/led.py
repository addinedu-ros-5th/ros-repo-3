from flask import Blueprint, request, Response
from http import HTTPStatus
import requests

led = Blueprint("led", __name__)
url = "http://192.168.137.204:80/led_control"

def request_esp(color):
    response = requests.post(url, params={"color": color})
    print("response text: " + response.text)
    print("response status: " + str(response.status_code))

@led.route("control", methods=["POST"])
def control():
    data = request.json
    status = data.get("status")
    
    if status == "Packing":
        request_esp("red")
    else:
        request_esp("green")
        
    return Response(None, HTTPStatus.OK)