#include "NoHighlightDelegate.h"

namespace rviz::panel::reconfigure {

NoHighlightDelegate::NoHighlightDelegate(QObject* parent)
    : QStyledItemDelegate(parent)
{
}

void NoHighlightDelegate::paint(QPainter* painter,
                                const QStyleOptionViewItem& option,
                                const QModelIndex& index) const
{
    QStyleOptionViewItem opt = option;
    opt.state &= ~QStyle::State_HasFocus;
    QStyledItemDelegate::paint(painter, opt, index);
}

}  // namespace rviz::panel::reconfigure
