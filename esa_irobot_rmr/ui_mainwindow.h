/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout_5;
    QFrame *frame1;
    QHBoxLayout *horizontalLayout_4;
    QSpacerItem *horizontalSpacer_2;
    QLabel *Warning_Prekazka_text;
    QSpacerItem *horizontalSpacer;
    QVBoxLayout *verticalLayout_4;
    QGroupBox *verticalGroupBox;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QLineEdit *lineEdit_Y;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *lineEdit_X;
    QHBoxLayout *horizontalLayout_3;
    QLabel *X_robot;
    QLineEdit *Xrobot;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_3;
    QLineEdit *Yrobot;
    QPushButton *pushButton_12;
    QCheckBox *Mapping;
    QCheckBox *checkBox_skeleton;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *showMap;
    QPushButton *showCam;
    QWidget *widget;
    QLabel *skeletonText;
    QFrame *robotShowSensors;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(937, 652);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        horizontalLayout_5 = new QHBoxLayout(centralWidget);
        horizontalLayout_5->setSpacing(0);
        horizontalLayout_5->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        horizontalLayout_5->setContentsMargins(0, 0, 0, 0);
        frame1 = new QFrame(centralWidget);
        frame1->setObjectName(QString::fromUtf8("frame1"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(60);
        sizePolicy.setVerticalStretch(50);
        sizePolicy.setHeightForWidth(frame1->sizePolicy().hasHeightForWidth());
        frame1->setSizePolicy(sizePolicy);
        frame1->setFrameShape(QFrame::StyledPanel);
        frame1->setFrameShadow(QFrame::Raised);
        horizontalLayout_4 = new QHBoxLayout(frame1);
        horizontalLayout_4->setSpacing(0);
        horizontalLayout_4->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_2);

        Warning_Prekazka_text = new QLabel(frame1);
        Warning_Prekazka_text->setObjectName(QString::fromUtf8("Warning_Prekazka_text"));
        Warning_Prekazka_text->setMinimumSize(QSize(500, 0));
        Warning_Prekazka_text->setMaximumSize(QSize(500, 50));
        Warning_Prekazka_text->setSizeIncrement(QSize(500, 0));
        QFont font;
        font.setPointSize(14);
        font.setBold(true);
        Warning_Prekazka_text->setFont(font);
        Warning_Prekazka_text->setStyleSheet(QString::fromUtf8("color: #f54242;\n"
"background-color: #FFF;\n"
"border: 2px solid #f54242;\n"
"border-radius: 2px;"));
        Warning_Prekazka_text->setAlignment(Qt::AlignCenter);

        horizontalLayout_4->addWidget(Warning_Prekazka_text);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalGroupBox = new QGroupBox(frame1);
        verticalGroupBox->setObjectName(QString::fromUtf8("verticalGroupBox"));
        verticalGroupBox->setMinimumSize(QSize(50, 0));
        verticalGroupBox->setMaximumSize(QSize(100, 300));
        verticalGroupBox->setLayoutDirection(Qt::RightToLeft);
        verticalGroupBox->setStyleSheet(QString::fromUtf8("background-color:rgba(0, 0, 0, 0.5);"));
        verticalLayout = new QVBoxLayout(verticalGroupBox);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_2 = new QLabel(verticalGroupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setStyleSheet(QString::fromUtf8("color:white;"));

        horizontalLayout_2->addWidget(label_2);

        lineEdit_Y = new QLineEdit(verticalGroupBox);
        lineEdit_Y->setObjectName(QString::fromUtf8("lineEdit_Y"));
        lineEdit_Y->setStyleSheet(QString::fromUtf8("color:white;"));
        lineEdit_Y->setReadOnly(false);

        horizontalLayout_2->addWidget(lineEdit_Y);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(verticalGroupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setStyleSheet(QString::fromUtf8("color:white;"));

        horizontalLayout->addWidget(label);

        lineEdit_X = new QLineEdit(verticalGroupBox);
        lineEdit_X->setObjectName(QString::fromUtf8("lineEdit_X"));
        lineEdit_X->setStyleSheet(QString::fromUtf8("color:white;"));
        lineEdit_X->setReadOnly(false);

        horizontalLayout->addWidget(lineEdit_X);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        X_robot = new QLabel(verticalGroupBox);
        X_robot->setObjectName(QString::fromUtf8("X_robot"));
        X_robot->setMaximumSize(QSize(15, 16777215));
        X_robot->setStyleSheet(QString::fromUtf8("color:white;"));

        horizontalLayout_3->addWidget(X_robot);

        Xrobot = new QLineEdit(verticalGroupBox);
        Xrobot->setObjectName(QString::fromUtf8("Xrobot"));
        Xrobot->setStyleSheet(QString::fromUtf8("color:white;"));
        Xrobot->setReadOnly(true);

        horizontalLayout_3->addWidget(Xrobot);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_3 = new QLabel(verticalGroupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setMinimumSize(QSize(15, 0));
        label_3->setMaximumSize(QSize(11, 16777215));
        label_3->setStyleSheet(QString::fromUtf8("color:white;\n"
""));

        horizontalLayout_9->addWidget(label_3);

        Yrobot = new QLineEdit(verticalGroupBox);
        Yrobot->setObjectName(QString::fromUtf8("Yrobot"));
        Yrobot->setMinimumSize(QSize(61, 0));
        Yrobot->setMaximumSize(QSize(61, 16777215));
        Yrobot->setStyleSheet(QString::fromUtf8("color:white;"));
        Yrobot->setReadOnly(true);

        horizontalLayout_9->addWidget(Yrobot);


        verticalLayout->addLayout(horizontalLayout_9);

        pushButton_12 = new QPushButton(verticalGroupBox);
        pushButton_12->setObjectName(QString::fromUtf8("pushButton_12"));
        pushButton_12->setStyleSheet(QString::fromUtf8("background-color:white;\n"
"color:black;"));

        verticalLayout->addWidget(pushButton_12);

        Mapping = new QCheckBox(verticalGroupBox);
        Mapping->setObjectName(QString::fromUtf8("Mapping"));
        Mapping->setStyleSheet(QString::fromUtf8("color: white; \n"
"background-color: rgba(0, 0, 0, 0.7);\n"
" border-style: outset; border-width: 1px; \n"
"border-color: beige"));

        verticalLayout->addWidget(Mapping);

        checkBox_skeleton = new QCheckBox(verticalGroupBox);
        checkBox_skeleton->setObjectName(QString::fromUtf8("checkBox_skeleton"));
        checkBox_skeleton->setStyleSheet(QString::fromUtf8("color: white; \n"
"background-color: rgba(0, 0, 0, 0.7); \n"
"border-style: outset; \n"
"border-width: 1px; \n"
"border-color: beige"));

        verticalLayout->addWidget(checkBox_skeleton);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        showMap = new QPushButton(verticalGroupBox);
        showMap->setObjectName(QString::fromUtf8("showMap"));
        showMap->setStyleSheet(QString::fromUtf8("color: white; \n"
"background-color: rgba(0, 0, 0, 0.7);\n"
"border-style: outset;\n"
"border-width: 1px;\n"
"border-color: beige;"));

        horizontalLayout_6->addWidget(showMap);

        showCam = new QPushButton(verticalGroupBox);
        showCam->setObjectName(QString::fromUtf8("showCam"));
        showCam->setStyleSheet(QString::fromUtf8("color: black; \n"
"background-color: white;\n"
"border-style: outset;\n"
"border-width: 1px;\n"
"border-color: beige;"));

        horizontalLayout_6->addWidget(showCam);


        verticalLayout->addLayout(horizontalLayout_6);


        verticalLayout_4->addWidget(verticalGroupBox);

        widget = new QWidget(frame1);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setMinimumSize(QSize(200, 30));
        widget->setMaximumSize(QSize(220, 35));
        skeletonText = new QLabel(widget);
        skeletonText->setObjectName(QString::fromUtf8("skeletonText"));
        skeletonText->setGeometry(QRect(0, 0, 200, 30));
        skeletonText->setMinimumSize(QSize(200, 30));
        skeletonText->setMaximumSize(QSize(200, 30));
        skeletonText->setFont(font);
        skeletonText->setLayoutDirection(Qt::LeftToRight);
        skeletonText->setStyleSheet(QString::fromUtf8("color: #000;\n"
"background-color: #FFF;\n"
"border: 2px solid #000;\n"
"border-radius: 2px;"));
        skeletonText->setAlignment(Qt::AlignCenter);

        verticalLayout_4->addWidget(widget);

        robotShowSensors = new QFrame(frame1);
        robotShowSensors->setObjectName(QString::fromUtf8("robotShowSensors"));
        robotShowSensors->setMinimumSize(QSize(200, 200));
        robotShowSensors->setMaximumSize(QSize(200, 200));
        robotShowSensors->setStyleSheet(QString::fromUtf8(""));
        robotShowSensors->setFrameShape(QFrame::StyledPanel);
        robotShowSensors->setFrameShadow(QFrame::Raised);

        verticalLayout_4->addWidget(robotShowSensors);


        horizontalLayout_4->addLayout(verticalLayout_4);


        horizontalLayout_5->addWidget(frame1);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        Warning_Prekazka_text->setText(QCoreApplication::translate("MainWindow", "V ceste je prek\303\241\305\276ka, naviguj na in\303\251 miesto", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Y:", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "X:", nullptr));
        X_robot->setText(QCoreApplication::translate("MainWindow", "Xr", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "Yr", nullptr));
        pushButton_12->setText(QCoreApplication::translate("MainWindow", "naviguj", nullptr));
        Mapping->setText(QCoreApplication::translate("MainWindow", "Mapping", nullptr));
        checkBox_skeleton->setText(QCoreApplication::translate("MainWindow", "skeleton", nullptr));
        showMap->setText(QCoreApplication::translate("MainWindow", "Map", nullptr));
        showCam->setText(QCoreApplication::translate("MainWindow", "Cam", nullptr));
        skeletonText->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
