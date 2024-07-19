from flask import Blueprint, request, Response
from connection.database import connection
from http import HTTPStatus

manager = Blueprint("manager", __name__)

@manager.route("customer_order", methods=["GET"])
def customer_order():
    data = request.json
    
    conn = connection()
    cursor = conn.cursor()
    
    for item in data["items"]:
        product_name = item.get("productName")
        product_amount = item.get("productAmount")
    
        query = "INSERT INTO customer_order (productName, productAmount) VALUES (%s, %s)"
        cursor.execute(query, (product_name, product_amount))
        conn.commit()

    cursor.close()
    conn.close()
    
    return Response(None, HTTPStatus.OK)