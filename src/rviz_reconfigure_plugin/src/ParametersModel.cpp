#include "ParametersModel.h"

#include <QBrush>

namespace rviz::panel::reconfigure {

ParametersModel::ParametersModel(QObject* parent)
    : QAbstractTableModel{parent}
{
}

int ParametersModel::rowCount(const QModelIndex&) const { return mParameters.size(); }

int ParametersModel::columnCount(const QModelIndex&) const { return 2; }

QVariant ParametersModel::data(const QModelIndex& index, int role) const
{
    QVariant value;
    if (index.isValid()) {
        auto parameter = mParameters.at(index.row());
        if (role == Qt::DisplayRole || role == Qt::EditRole) {
            switch (index.column()) {
            case 0:
                value = parameter.getName();
                break;
            case 1:
                if (parameter.getValue().type() == QVariant::Double) {
                    value = QString::number(mParameters.at(index.row()).getValue().toDouble());
                }
                else {
                    value = mParameters.at(index.row()).getValue();
                }
                break;
            }
        }

        if (role == Qt::ForegroundRole) {
            return parameter.isChanged() ? QBrush(Qt::blue) : QBrush(Qt::black);
        }
    }

    return value;
}

Qt::ItemFlags ParametersModel::flags(const QModelIndex& index) const
{
    auto flags = QAbstractTableModel::flags(index);
    if (index.isValid()) {
        auto parameter = mParameters.at(index.row());
        if (index.column() == 1) {
            if (parameter.isReadonly()) {
                flags = flags & ~Qt::ItemIsEnabled;
            }
            else {
                flags = flags | Qt::ItemIsEditable;
            }
        }
    }

    return flags;
}

bool ParametersModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if (index.isValid() && role == Qt::EditRole && index.column() == 1) {
        auto& parameter = mParameters[index.row()];
        if (value.type() == parameter.getValue().type() ||
            parameter.getValue().type() == QVariant::Double) {
            if (parameter.getValue().type() == QVariant::Double) {
                bool isOk;
                auto number = value.toDouble(&isOk);
                if (isOk) {
                    parameter.setData(number);
                    emit dataChanged(index, index);
                    emit parameterChanged(parameter.getName(), number);
                    return true;
                }
            }
            else {
                parameter.setData(value);
                emit dataChanged(index, index);
                emit parameterChanged(parameter.getName(), value);
                return true;
            }
        }
    }

    return false;
}

void ParametersModel::onParametersReceived(QVariant varParams)
{
    beginResetModel();
    auto parameters = varParams.value<QList<ParameterItem>>();
    mParameters = QList<ParameterItemEx>{parameters.begin(), parameters.end()};

    endResetModel();
}

void ParametersModel::restoreData(const QModelIndex& index)
{
    if (index.isValid()) {
        mParameters[index.row()].restore();
        setData(index, mParameters[index.row()].getValue(), Qt::EditRole);
    }
}

QVariant ParametersModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    QVariant variant;
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole) {
        variant = section == 0 ? "Name" : "Value";
    }

    return variant;
}

}  // namespace rviz::panel::reconfigure

