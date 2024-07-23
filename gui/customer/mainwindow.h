#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <button.h>
#include <shopping_cart.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow, Button, ShoppingCart
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onButtonClicked();

private:
    Ui::MainWindow *ui;

public:
    void clearWidget();
};
#endif // MAINWINDOW_H
