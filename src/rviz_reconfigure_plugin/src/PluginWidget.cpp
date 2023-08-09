#include "PluginWidget.h"

#include <QAction>
#include <QApplication>
#include <QClipboard>
#include <QCloseEvent>
#include <QList>
#include <QMenu>

#include "ui_PluginWidget.h"

#include "NoHighlightDelegate.h"

namespace rviz::panel::reconfigure {

PluginWidget::PluginWidget(QWidget* parent)
    : QWidget(parent)
    , mUi{new Ui::PluginWidget}
{
    mUi->setupUi(this);

    mUi->propertiesTableView->setItemDelegate(new NoHighlightDelegate(mUi->propertiesTableView));

    mParametersProxyModel.setSourceModel(&mParametersModel);
    mParametersProxyModel.setFilterKeyColumn(0);
    mNodesProxyModel.setSourceModel(&mNodesModel);
    mNodesProxyModel.setFilterKeyColumn(0);

    mUi->nodesTableView->setModel(&mNodesProxyModel);
    mUi->propertiesTableView->setModel(&mParametersProxyModel);

    auto nodesSelectionModel = mUi->nodesTableView->selectionModel();

    connect(mUi->refreshButton, &QPushButton::clicked, this, &PluginWidget::onRefreshClicked);
    connect(nodesSelectionModel, &QItemSelectionModel::selectionChanged, this,
            &PluginWidget::onNodeChanged);

    connect(&mRclEngine, &RclEngine::nodeReceived, &mNodesModel, &NodeModel::onNodeReceived);
    connect(&mRclEngine, &RclEngine::parametersReceived, &mParametersModel,
            &ParametersModel::onParametersReceived);
    connect(&mRclEngine, &RclEngine::parametersReceived, this, &PluginWidget::onParametersReceived);
    connect(&mRclEngine, &RclEngine::parameterUpdated, this, &PluginWidget::onParameterUpdated);
    connect(mUi->propertyFilterLine, &QLineEdit::returnPressed, this,
            &PluginWidget::onParametersFilter);
    connect(mUi->propertyFilterButton, &QPushButton::clicked, this,
            &PluginWidget::onParametersFilter);
    connect(mUi->nodeFilterLine, &QLineEdit::returnPressed, this, &PluginWidget::onNodeFilter);
    connect(mUi->nodeFilterButton, &QPushButton::clicked, this, &PluginWidget::onNodeFilter);
    connect(mUi->propertiesTableView, &QTableView::customContextMenuRequested, this,
            &PluginWidget::showContextMenu);
    connect(&mParametersModel, &ParametersModel::parameterChanged, this,
            &PluginWidget::onParameterChanged);
}

void PluginWidget::setNode(rclcpp::Node::SharedPtr node) { mNode = node; }

void PluginWidget::onRefreshClicked()
{
    mRclEngine.enumerateNodes();
    mCurrentNodeName.clear();
}

void PluginWidget::onNodeChanged(const QItemSelection& selected, const QItemSelection&)
{
    if (selected.indexes().size() > 0) {
        auto newModelIndex = selected.indexes().at(0);
        QString nodeName = mNodesProxyModel.data(newModelIndex).toString();
        mRclEngine.enumerateParameters(nodeName);
        mUi->nodeLabel->setText(nodeName);
        mCurrentNodeName = nodeName;
    }
}

void PluginWidget::onParametersFilter()
{
    auto pattern = mUi->propertyFilterLine->text();
    mParametersProxyModel.setFilterRegExp(
        QRegExp(pattern, Qt::CaseInsensitive, QRegExp::FixedString));
}

void PluginWidget::onNodeFilter()
{
    auto pattern = mUi->nodeFilterLine->text();
    mNodesProxyModel.setFilterRegExp(QRegExp(pattern, Qt::CaseInsensitive, QRegExp::FixedString));
}

void PluginWidget::closeEvent(QCloseEvent* event)
{
    rclcpp::shutdown();
    event->accept();
}

void PluginWidget::showContextMenu(const QPoint& pos)
{
    QModelIndex index = mUi->propertiesTableView->indexAt(pos);
    if (index.isValid()) {
        QMenu contextMenu;
        QAction restoreAction("Restore value", this);
        QAction copyNameAction("Copy name", this);
        QAction copyValueAction("Copy value", this);
        QAction copyNameAndValueAction("Copy name and value", this);
        QAction copyAllAction("Copy all names and values", this);

        contextMenu.addAction(&restoreAction);
        contextMenu.addAction(&copyNameAction);
        contextMenu.addAction(&copyValueAction);
        contextMenu.addAction(&copyNameAndValueAction);
        contextMenu.addAction(&copyAllAction);

        connect(&restoreAction, &QAction::triggered,
                [&model = mParametersProxyModel, &index, &sourceModel = mParametersModel]() {
                    auto sourceIndex = model.mapToSource(index);
                    sourceModel.restoreData(sourceIndex);
                });

        connect(&copyNameAction, &QAction::triggered, [&model = mParametersProxyModel, &index]() {
            auto dataIndex = model.index(index.row(), 0);
            auto value = model.data(dataIndex).toString();
            qDebug() << value;
            QApplication::clipboard()->setText(value);
        });

        connect(&copyValueAction, &QAction::triggered, [&model = mParametersProxyModel, &index]() {
            auto dataIndex = model.index(index.row(), 1);
            auto value = model.data(dataIndex).toString();
            qDebug() << value;
            QApplication::clipboard()->setText(value);
        });

        connect(&copyNameAndValueAction, &QAction::triggered,
                [&model = mParametersProxyModel, &index]() {
                    auto nameIndex = model.index(index.row(), 0);
                    auto valueIndex = model.index(index.row(), 1);
                    auto name = model.data(nameIndex).toString();
                    auto value = model.data(valueIndex).toString();
                    qDebug() << name << value;
                    QApplication::clipboard()->setText(
                        QString("\"%1\":\"%2\"").arg(name).arg(value));
                });

        connect(&copyAllAction, &QAction::triggered, []() { qDebug() << "copyAllAction"; });

        contextMenu.exec(mUi->propertiesTableView->viewport()->mapToGlobal(pos));
    }
}

void PluginWidget::onParameterChanged(const QString& name, const QVariant& value)
{
    mRclEngine.setParameterValue(mCurrentNodeName, name, value);
}

void PluginWidget::onParameterUpdated(const QString&)
{
    //    mRclEngine.enumerateParameters(mCurrentNodeName);
}

void PluginWidget::onParametersReceived(QVariant)
{
    mUi->propertiesTableView->resizeColumnToContents(0);
}

}  // namespace rviz::panel::reconfigure
