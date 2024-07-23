from flask import Blueprint, request, Response
from connection.database import connection
from ros.publisher import pub
from outbound_delivery_robot_interfaces.msg import Location
from http import HTTPStatus

location = Blueprint("location", __name__)
location_publisher = pub()

@location.route("coordinate", methods=["POST"])
def get_location():
    product = request.args.get("product")
    conn = connection()
    cursor = conn.cursor()
    
    query = f"SELECT * FROM location WHERE product= %s"
    cursor.execute(query, (product, ))
    
    result = cursor.fetchall()[0]
    print(result)
    
    cursor.close()
    conn.close()
    
    location = Location()
    location.section = result[1]
    location.x = result[2]
    location.y = result[3]
    location.z = result[4]
    location.w = result[5]
    
    location_publisher.publish_location(location)
    
    return Response(None, HTTPStatus.OK)