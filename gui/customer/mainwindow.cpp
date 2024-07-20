#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->stackedWidget->setCurrentIndex(0);

    connect(ui->cartBtn, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->backBtn, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->plusBtn, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->plusBtn_2, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->plusBtn_3, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->plusBtn_4, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->minusBtn, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->minusBtn_2, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->minusBtn_3, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->minusBtn_4, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->addBtn, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->addBtn_2, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->addBtn_3, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->addBtn_4, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->clearBtn, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->buyBtn, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
}

MainWindow::~MainWindow()
{
    qDeleteAll(cartItems);
    delete ui;
}

void MainWindow::plusAmount(QLabel& productCount)
{
    int count = productCount.text().toInt();
    if (count < 5)
    {
        count += 1;
        productCount.setText(QString::number(count));
    }
}

void MainWindow::minusAmount(QLabel& productCount)
{
    int count = productCount.text().toInt();
    if (count > 1)
    {
        count -= 1;
        productCount.setText(QString::number(count));
    }
}

QMap<QString, QWidget*> MainWindow::getCartItems() const
{
    return cartItems;
}

void MainWindow::setCartItems(const QMap<QString, QWidget*> &items)
{
    cartItems = items;
}

void MainWindow::addItem(QLabel& image, QLabel& name, QLabel& amount, QLabel& price)
{
    QMap<QString, QWidget*> items = getCartItems();
    QString itemName = name.text();

    if (items.contains(itemName))
    {
        QWidget* itemWidget = items[itemName];
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
        QWidget* itemWidget = new QWidget;
        QHBoxLayout* itemLayout = new QHBoxLayout(itemWidget);

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

        ui->cartVBoxLayout->addWidget(itemWidget);

        items[itemName] = itemWidget;
    }
    setCartItems(items);
}

void MainWindow::clearItem()
{
    QMap<QString, QWidget*> items = getCartItems();
    for (auto& item : items)
    {
        ui->cartVBoxLayout->removeWidget(item);
        delete item;
    }
    items.clear();
    setCartItems(items);
}

void MainWindow::uploadItem()
{
    QJsonArray itemsArray;
    for (auto item : cartItems.keys())
    {
        QWidget* itemWidget = cartItems[item];
        QLabel* amountLabel = itemWidget->findChild<QLabel*>("amountLabel");

        QJsonObject itemObject;
        itemObject["productName"] = item;
        itemObject["productAmount"] = amountLabel->text().toInt();

        itemsArray.append(itemObject);
    }

    QJsonObject json;
    json["items"] = itemsArray;

    QJsonDocument jsonDoc(json);
    QByteArray jsonData = jsonDoc.toJson();

    QNetworkAccessManager *manager = new QNetworkAccessManager(this);
    QNetworkRequest request(QUrl("http://address:port/manager/customer_order"));
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    QNetworkReply* reply = manager->post(request, jsonData);

    connect(reply, &QNetworkReply::finished, this, [this, reply]()
    {
        if (reply->error() == QNetworkReply::NoError)
        {
            qDebug() << "Data uploaded successfully";
            clearItem();
        }
        else
        {
            qDebug() << "Error uploading data: " << reply->errorString();
        }

        reply->deleteLater();
    });
    qDebug() << "Sending request to server...";
}

void MainWindow::onButtonClicked()
{
    QPushButton* clickedButton = qobject_cast<QPushButton*>(sender());
    if(!clickedButton) return;

    if (clickedButton == ui->cartBtn)
    {
        ui->stackedWidget->setCurrentIndex(1);
    }
    else if (clickedButton == ui->backBtn)
    {
        ui->stackedWidget->setCurrentIndex(0);
    }
    else if (clickedButton == ui->plusBtn)
    {
        plusAmount(*ui->goodsCount);
    }
    else if (clickedButton == ui->plusBtn_2)
    {
        plusAmount(*ui->goodsCount_2);
    }
    else if (clickedButton == ui->plusBtn_3)
    {
        plusAmount(*ui->goodsCount_3);
    }
    else if (clickedButton == ui->plusBtn_4)
    {
        plusAmount(*ui->goodsCount_4);
    }
    else if (clickedButton == ui->minusBtn)
    {
        minusAmount(*ui->goodsCount);
    }
    else if (clickedButton == ui->minusBtn_2)
    {
        minusAmount(*ui->goodsCount_2);
    }
    else if (clickedButton == ui->minusBtn_3)
    {
        minusAmount(*ui->goodsCount_3);
    }
    else if (clickedButton == ui->minusBtn_4)
    {
        minusAmount(*ui->goodsCount_4);
    }
    else if (clickedButton == ui->addBtn)
    {
        addItem(*ui->goodsImage, *ui->goodsName, *ui->goodsCount, *ui->goodsPrice);
    }
    else if (clickedButton == ui->addBtn_2)
    {
        addItem(*ui->goodsImage_2, *ui->goodsName_2, *ui->goodsCount_2, *ui->goodsPrice_2);
    }
    else if (clickedButton == ui->addBtn_3)
    {
        addItem(*ui->goodsImage_3, *ui->goodsName_3, *ui->goodsCount_3, *ui->goodsPrice_3);
    }
    else if (clickedButton == ui->addBtn_4)
    {
        addItem(*ui->goodsImage_4, *ui->goodsName_4, *ui->goodsCount_4, *ui->goodsPrice_4);
    }
    else if (clickedButton == ui->buyBtn)
    {
        uploadItem();
    }
    else if (clickedButton == ui->clearBtn)
    {
        clearItem();
    }
}