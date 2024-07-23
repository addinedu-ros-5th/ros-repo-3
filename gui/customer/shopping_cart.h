#ifndef SHOPPING_CART_H
#define SHOPPING_CART_H

#include <QMap>
#include <QLabel>
#include <QWidget>
#include <QNetworkReply>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <QHBoxLayout>
#include <QSharedPointer>

class ShoppingCart
{
public:
  void setCartItems(const QMap<QString, QSharedPointer<QWidget>> &items);
  QMap<QString, QSharedPointer<QWidget>> getCartItems() const;
  QSharedPointer<QWidget> addItem(QLabel& image, QLabel& name, QLabel& amount, QLabel& price);
  void clearItem();
  QSharedPointer<QNetworkReply> uploadItem();

protected:
  QMap<QString, QSharedPointer<QWidget>> cartItems;

private:
  QSharedPointer<QNetworkAccessManager> manager;
};

#endif // SHOPPING_CART_H
