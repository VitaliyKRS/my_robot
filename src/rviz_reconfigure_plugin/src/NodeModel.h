#pragma once

#include <QAbstractListModel>

namespace rviz::panel::reconfigure {

class NodeModel : public QAbstractListModel {
    Q_OBJECT
private:
    QStringList mNodeNames;

public:
    NodeModel(QObject* parent = nullptr);

public:
    int rowCount(const QModelIndex&) const;

    QString getNodeName(uint32_t index) const;

    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;

public slots:
    void onNodeReceived(QStringList nodeNames);
};

}  // namespace rviz::panel::reconfigure
