#pragma once

#include <QAbstractTableModel>

#include "RclEngine.h"

namespace rviz::panel::reconfigure {

class ParameterItemEx : public ParameterItem {
private:
    QVariant mOriginalValue;

public:
    ParameterItemEx(const ParameterItem& item)
        : ParameterItem{item}
        , mOriginalValue{item.getValue()}
    {
    }

public:
    inline bool isChanged() { return mOriginalValue != getValue(); }

    inline void restore() { setData(mOriginalValue); }
};

class ParametersModel : public QAbstractTableModel {
    Q_OBJECT
private:
    QList<ParameterItemEx> mParameters;

public:
    explicit ParametersModel(QObject* parent = nullptr);

public:  // QAbstractTableModel
    int rowCount(const QModelIndex&) const override;

    int columnCount(const QModelIndex&) const override;

    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;

    Qt::ItemFlags flags(const QModelIndex& index) const override;

    bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;

    QVariant headerData(int section,
                        Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override;

public:
    void restoreData(const QModelIndex& index);

public slots:
    void onParametersReceived(QVariant parameters);

signals:
    void parameterChanged(const QString& name, const QVariant& value);
};

}  // namespace rviz::panel::reconfigure
