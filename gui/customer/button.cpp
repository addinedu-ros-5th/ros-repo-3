#include <button.h>

void Button::plusQuantity(QLabel &product)
{
    int count = product.text().toInt();
    if (count < 5)
    {
        count += 1;
        product.setText(QString::number(count));
    }
}

void Button::minusQuantity(QLabel &product)
{
    int count = product.text().toInt();
    if (count > 1)
    {
        count -= 1;
        product.setText(QString::number(count));
    }
}
