from flask import Blueprint, request, Response
from http import HTTPStatus

camera = Blueprint("camera", __name__)

@camera.route("/detection", methods=["POST"])
def detection():
    data = request.json
    box_detection = data.get("boxes")
    robot_detection = data.get("robots")
    person_detection = data.get("persons")
    
    if box_detection == "1":
        pass
    else:
        pass
    
    if robot_detection == "1":
        pass
    else:
        pass
    
    if person_detection == "1":
        pass
    else:
        pass

    return Response(None, HTTPStatus.OK)