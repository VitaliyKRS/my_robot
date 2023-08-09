#include "NodeModel.h"

namespace rviz::panel::reconfigure {

NodeModel::NodeModel(QObject* parent)
    : QAbstractListModel{parent}
{
}

int NodeModel::rowCount(const QModelIndex&) const { return mNodeNames.size(); }

QString NodeModel::getNodeName(uint32_t index) const { return mNodeNames.at(index); }

QVariant NodeModel::data(const QModelIndex& index, int role) const
{
    if (index.isValid()) {
        if (role == Qt::DisplayRole) {
            return mNodeNames[index.row()];
        }
    }

    return {};
}

void NodeModel::onNodeReceived(QStringList nodeNames)
{
    beginResetModel();
    mNodeNames = nodeNames;
    endResetModel();
}

}  // namespace rviz::panel::reconfigure
