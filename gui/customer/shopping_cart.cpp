#include <shopping_cart.h>

QSharedPointer<QWidget> ShoppingCart::addItem(QLabel& image, QLabel& name, QLabel& amount, QLabel& price)
{
    QMap<QString, QSharedPointer<QWidget>> items = getCartItems();
    QString itemName = name.text();

    QSharedPointer<QWidget> itemWidget;

    if (items.contains(itemName))
    {
        itemWidget = items[itemName];
        QLabel* amountLabel = itemWidget->findChild<QLabel*>("amountLabel");
        QLabel* totalPriceLabel = itemWidget->findChild<QLabel*>("totalPriceLabel");

        int currentAmount = amountLabel->text().toInt();
        int newAmount = currentAmount + amount.text().toInt();
        if (newAmount <= 5)
        {
            amountLabel->setText(QString::number(newAmount));
            int newTotalPrice = price.text().toInt() * newAmount;
            totalPriceLabel->setText(QString::number(newTotalPrice));
        }
    }
    else
    {
        itemWidget = QSharedPointer<QWidget>::create();
        QHBoxLayout* itemLayout = new QHBoxLayout(itemWidget.data());

        QLabel* imageCopy = new QLabel;
        imageCopy->setStyleSheet(image.styleSheet());
        imageCopy->setMinimumSize(150, 150);
        imageCopy->setMaximumSize(150, 150);

        QLabel* nameCopy = new QLabel;
        nameCopy->setText(name.text());
        nameCopy->setAlignment(Qt::AlignCenter);

        QLabel* amountCopy = new QLabel;
        amountCopy->setObjectName("amountLabel");
        amountCopy->setText(amount.text());
        amountCopy->setAlignment(Qt::AlignCenter);

        QLabel* totalPrice = new QLabel;
        totalPrice->setObjectName("totalPriceLabel");
        int total = price.text().toInt() * amount.text().toInt();
        totalPrice->setText(QString::number(total));
        totalPrice->setAlignment(Qt::AlignCenter);

        itemLayout->addWidget(imageCopy);
        itemLayout->addWidget(nameCopy);
        itemLayout->addWidget(amountCopy);
        itemLayout->addWidget(totalPrice);

        items[itemName] = itemWidget;
        setCartItems(items);
    }
    return itemWidget;
}

void ShoppingCart::clearItem()
{
    QMap<QString, QSharedPointer<QWidget>> items = getCartItems();
    items.clear();
    setCartItems(items);
}

QSharedPointer<QNetworkReply> ShoppingCart::uploadItem()
{
    QJsonArray itemsArray;
    for (auto item : cartItems.keys())
    {
        QSharedPointer<QWidget> itemWidget = cartItems[item];
        QLabel* amountLabel = itemWidget->findChild<QLabel*>("amountLabel");
        QLabel* priceLabel = itemWidget->findChild<QLabel*>("totalPriceLabel");

        QJsonObject itemObject;
        itemObject["productName"] = item;
        itemObject["productAmount"] = amountLabel->text().toInt();
        itemObject["productPrice"] = priceLabel->text().toInt();

        itemsArray.append(itemObject);
    }

    QJsonObject json;
    json["items"] = itemsArray;

    QString orderTime = QDateTime::currentDateTime().toString("yyyy-MM-dd");
    json["orderTime"] = orderTime;

    QJsonDocument jsonDoc(json);
    QByteArray jsonData = jsonDoc.toJson();

    if (!manager)
    {
        manager = QSharedPointer<QNetworkAccessManager>::create();
    }
    QNetworkRequest request(QUrl("http://address:port/manager/customer_order"));
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    QSharedPointer<QNetworkReply> reply(manager->post(request, jsonData));
    return reply;
}

QMap<QString, QSharedPointer<QWidget>> ShoppingCart::getCartItems() const
{
    return cartItems;
}

void ShoppingCart::setCartItems(const QMap<QString, QSharedPointer<QWidget>> &items)
{
    cartItems = items;
}
