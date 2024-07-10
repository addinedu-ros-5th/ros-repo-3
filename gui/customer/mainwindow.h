#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QMap>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onButtonClicked();
    void minusAmount(QLabel& count);
    void plusAmount(QLabel& count);
    void addItem(QLabel& image, QLabel& name, QLabel& amount, QLabel& price);

private:
    Ui::MainWindow *ui;
    QMap<QString, QWidget*> cartItems;
};
#endif // MAINWINDOW_H
