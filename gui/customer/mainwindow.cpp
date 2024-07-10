#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
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

void MainWindow::addItem(QLabel& image, QLabel& name, QLabel& amount, QLabel& price)
{
    QString itemName = name.text();

    if (cartItems.contains(itemName))
    {
        QWidget* itemWidget = cartItems[itemName];
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
    else {
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

        cartItems[itemName] = itemWidget;
    }
}

void MainWindow::onButtonClicked()
{
    QPushButton* clickedButton = qobject_cast<QPushButton*>(sender());
    if(clickedButton)
    {
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
            //TODO
        }
    }
}
