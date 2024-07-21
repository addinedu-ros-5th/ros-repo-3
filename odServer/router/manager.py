from flask import Blueprint, request, Response, jsonify
from connection.database import connection
from http import HTTPStatus

manager = Blueprint("manager", __name__)

@manager.route("customer_order", methods=["POST"])
def customer_order():
    data = request.json
    items = data.get("items", [])
    order_time = data.get("orderTime", None)
    
    conn = connection()
    cursor = conn.cursor()

    try:
        query = "INSERT INTO orders (order_time) VALUES (%s)"
        cursor.execute(query, (order_time, ))
        order_id = cursor.lastrowid
    
        for item in items:
            product_name = item.get("productName")
            quantity = item.get("productAmount")
            price = item.get("productPrice")
            query = "INSERT INTO order_details (order_id, product_name, quantity, price) VALUES (%s, %s, %s, %s)"
            cursor.execute(query, (order_id, product_name, quantity, price))
        
        conn.commit()

    except:
        conn.rollback()

    finally:
        cursor.close()
        conn.close()
    
    return Response(None, HTTPStatus.OK)

@manager.route("order", methods=["GET"])
def order():
    date = request.args.get("date")

    conn = connection()
    cursor = conn.cursor()

    query = f"SELECT order_id, order_time \
             FROM orders \
             WHERE order_time={date} \
             ORDER BY order_id"

    try:
        cursor.execute(query)
        result = cursor.fetchall()
        response = {"order": result}
        return jsonify(response)
    finally:
        cursor.close()
        conn.close()

@manager.route("order_detail", methods=["GET"])
def order_detail():
    conn = connection()
    cursor = conn.cursor()

    query = "SELECT d.product_name, d.quantity \
            FROM orders o \
            JOIN order_details d ON o.order_id = d.order_id \
            WHERE o.order_id = %s \
            ORDER BY o.order_time"

    try:
        cursor.execute(query)
        result = cursor.fetchall()
        response = {"order_detail": result}
        return jsonify(response)
    finally:
        cursor.close()
        conn.close()
