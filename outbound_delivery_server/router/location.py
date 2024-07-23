from flask import Blueprint, request, jsonify
from connection.database import connection
from ros.outbound_pub import init

location = Blueprint("location", __name__)
outbound_publisher = init()

@location.route("coordinate", methods=["POST"])
def get_location():
    product = request.args.get("product")
    conn = connection()
    cursor = conn.cursor()
    
    query = f"SELECT * FROM location WHERE product={product}"
    cursor.execute(query)
    
    result = cursor.fetchall()
    response = {"location": result}

    cursor.close()
    conn.close()
    
    return jsonify(response)