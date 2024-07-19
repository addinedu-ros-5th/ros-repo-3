from flask import Blueprint, request, Response
from http import HTTPStatus
from ros.outbound_pub import init

reader = Blueprint("reader", __name__)
outbound_publisher = init()

@reader.route("product", methods=["POST"])
def read_tag():
    data = request.json
    status = data.get("tag_data")
    print(status)
    # if status == "Success":
    #     outbound_publisher.publish_message(status)
    return Response(None, HTTPStatus.OK)