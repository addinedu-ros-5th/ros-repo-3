from flask import Blueprint, request, Response
from http import HTTPStatus
from ros.publisher import pub
from std_msgs.msg import String


camera = Blueprint("camera", __name__)

detection_publisher = pub()

@camera.route("/detection", methods=["POST"])
def detection():
    data = request.json
    box_detection = data.get("boxes")
    robot_detection = data.get("robots")
    person_detection = data.get("persons")
    
    if box_detection == "1":
        msg = String()
        msg.data = "person"
        detection_publisher.publish_detection(msg)
    else:
        pass
    
    if robot_detection == "1":
        msg = String()
        msg.data = "robot"
        detection_publisher.publish_detection(msg)
    else:
        pass
    
    if person_detection == "1":
        msg = String()
        msg.data = "robot"
        detection_publisher.publish_detection(msg)
    else:
        pass

    return Response(None, HTTPStatus.OK)