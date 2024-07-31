#include <mainwindow.h>
#include <ui_mainwindow.h>
#include <QDebug>
#include <QPointer>
#include <QSharedPointer>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->stackedWidget->setCurrentIndex(0);

    connect(ui->cartBtn, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->backBtn, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->plusBtn, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->plusBtn_2, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->plusBtn_3, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->plusBtn_4, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->plusBtn_5, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->plusBtn_6, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->plusBtn_7, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->plusBtn_8, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->minusBtn, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->minusBtn_2, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->minusBtn_3, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->minusBtn_4, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->minusBtn_5, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->minusBtn_6, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->minusBtn_7, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->minusBtn_8, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->addBtn, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->addBtn_2, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->addBtn_3, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->addBtn_4, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->addBtn_5, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->addBtn_6, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->addBtn_7, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->addBtn_8, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->clearBtn, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->buyBtn, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
}

MainWindow::~MainWindow()
{
    clearWidget();
    delete ui;
}

void MainWindow::clearWidget()
{
    cartItems.clear();
}

void MainWindow::onButtonClicked()
{
    QPushButton* clickedButton = qobject_cast<QPushButton*>(sender());
    if (!clickedButton) return;

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
        plusQuantity(*ui->goodsCount);
    }
    else if (clickedButton == ui->plusBtn_2)
    {
        plusQuantity(*ui->goodsCount_2);
    }
    else if (clickedButton == ui->plusBtn_3)
    {
        plusQuantity(*ui->goodsCount_3);
    }
    else if (clickedButton == ui->plusBtn_4)
    {
        plusQuantity(*ui->goodsCount_4);
    }
    else if (clickedButton == ui->plusBtn_5)
    {
        plusQuantity(*ui->goodsCount_5);
    }
    else if (clickedButton == ui->plusBtn_6)
    {
        plusQuantity(*ui->goodsCount_6);
    }
    else if (clickedButton == ui->plusBtn_7)
    {
        plusQuantity(*ui->goodsCount_7);
    }
    else if (clickedButton == ui->plusBtn_8)
    {
        plusQuantity(*ui->goodsCount_8);
    }
    else if (clickedButton == ui->minusBtn)
    {
        minusQuantity(*ui->goodsCount);
    }
    else if (clickedButton == ui->minusBtn_2)
    {
        minusQuantity(*ui->goodsCount_2);
    }
    else if (clickedButton == ui->minusBtn_3)
    {
        minusQuantity(*ui->goodsCount_3);
    }
    else if (clickedButton == ui->minusBtn_4)
    {
        minusQuantity(*ui->goodsCount_4);
    }
    else if (clickedButton == ui->minusBtn_5)
    {
        minusQuantity(*ui->goodsCount_5);
    }
    else if (clickedButton == ui->minusBtn_6)
    {
        minusQuantity(*ui->goodsCount_6);
    }
    else if (clickedButton == ui->minusBtn_7)
    {
        minusQuantity(*ui->goodsCount_7);
    }
    else if (clickedButton == ui->minusBtn_8)
    {
        minusQuantity(*ui->goodsCount_8);
    }
    else if (clickedButton == ui->addBtn)
    {
        auto itemWidget = QSharedPointer<QWidget>(addItem(*ui->goodsImage, *ui->goodsName, *ui->goodsCount, *ui->goodsPrice));
        ui->cartVBoxLayout->addWidget(itemWidget.data(), 0, Qt::Alignment(Qt::AlignTop));
        cartItems.insert(ui->goodsName->text(), itemWidget);
    }
    else if (clickedButton == ui->addBtn_2)
    {
        auto itemWidget = QSharedPointer<QWidget>(addItem(*ui->goodsImage_2, *ui->goodsName_2, *ui->goodsCount_2, *ui->goodsPrice_2));
        ui->cartVBoxLayout->addWidget(itemWidget.data(), 0, Qt::Alignment(Qt::AlignTop));
        cartItems.insert(ui->goodsName_2->text(), itemWidget);
    }
    else if (clickedButton == ui->addBtn_3)
    {
        auto itemWidget = QSharedPointer<QWidget>(addItem(*ui->goodsImage_3, *ui->goodsName_3, *ui->goodsCount_3, *ui->goodsPrice_3));
        ui->cartVBoxLayout->addWidget(itemWidget.data(), 0, Qt::Alignment(Qt::AlignTop));
        cartItems.insert(ui->goodsName_3->text(), itemWidget);
    }
    else if (clickedButton == ui->addBtn_4)
    {
        auto itemWidget = QSharedPointer<QWidget>(addItem(*ui->goodsImage_4, *ui->goodsName_4, *ui->goodsCount_4, *ui->goodsPrice_4));
        ui->cartVBoxLayout->addWidget(itemWidget.data(), 0, Qt::Alignment(Qt::AlignTop));
        cartItems.insert(ui->goodsName_4->text(), itemWidget);
    }
    else if (clickedButton == ui->addBtn_5)
    {
        auto itemWidget = QSharedPointer<QWidget>(addItem(*ui->goodsImage_5, *ui->goodsName_5, *ui->goodsCount_5, *ui->goodsPrice_5));
        ui->cartVBoxLayout->addWidget(itemWidget.data(), 0, Qt::Alignment(Qt::AlignTop));
        cartItems.insert(ui->goodsName_5->text(), itemWidget);
    }
    else if (clickedButton == ui->addBtn_6)
    {
        auto itemWidget = QSharedPointer<QWidget>(addItem(*ui->goodsImage_6, *ui->goodsName_6, *ui->goodsCount_6, *ui->goodsPrice_6));
        ui->cartVBoxLayout->addWidget(itemWidget.data(), 0, Qt::Alignment(Qt::AlignTop));
        cartItems.insert(ui->goodsName_6->text(), itemWidget);
    }
    else if (clickedButton == ui->addBtn_7)
    {
        auto itemWidget = QSharedPointer<QWidget>(addItem(*ui->goodsImage_7, *ui->goodsName_7, *ui->goodsCount_7, *ui->goodsPrice_7));
        ui->cartVBoxLayout->addWidget(itemWidget.data(), 0, Qt::Alignment(Qt::AlignTop));
        cartItems.insert(ui->goodsName_7->text(), itemWidget);
    }
    else if (clickedButton == ui->addBtn_8)
    {
        auto itemWidget = QSharedPointer<QWidget>(addItem(*ui->goodsImage_8, *ui->goodsName_8, *ui->goodsCount_8, *ui->goodsPrice_8));
        ui->cartVBoxLayout->addWidget(itemWidget.data(), 0, Qt::Alignment(Qt::AlignTop));
        cartItems.insert(ui->goodsName_8->text(), itemWidget);
    }
    else if (clickedButton == ui->buyBtn)
    {
        auto reply = QSharedPointer<QNetworkReply>(uploadItem());
        connect(reply.data(), &QNetworkReply::finished, this, [this, reply]()
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
        });
    }
    else if (clickedButton == ui->clearBtn)
    {
        clearWidget();
        clearItem();
    }
}
