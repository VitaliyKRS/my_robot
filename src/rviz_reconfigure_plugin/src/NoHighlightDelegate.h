#pragma once

#include <QStyledItemDelegate>

namespace rviz::panel::reconfigure {

class NoHighlightDelegate : public QStyledItemDelegate {
public:
    NoHighlightDelegate(QObject* parent = nullptr);

public:  // QStyledItemDelegate
    void paint(QPainter* painter,
               const QStyleOptionViewItem& option,
               const QModelIndex& index) const override;
};

}  // namespace rviz::panel::reconfigure
