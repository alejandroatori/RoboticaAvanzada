/********************************************************************************
** Form generated from reading UI file 'mainUI.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINUI_H
#define UI_MAINUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QVBoxLayout *verticalLayout;
    QSplitter *splitter;
    QFrame *beta_frame;
    QFrame *control_frame;
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QLabel *top_camera_label;
    QHBoxLayout *horizontalLayout_2;
    QHBoxLayout *horizontalLayout;
    QLCDNumber *speed_lcd;
    QLabel *speed_label;
    QHBoxLayout *horizontalLayout_4;
    QHBoxLayout *horizontalLayout_3;
    QLabel *posx_label;
    QSpacerItem *horizontalSpacer;
    QLCDNumber *posx_lcd;
    QHBoxLayout *horizontalLayout_5;
    QLabel *posz_label;
    QSpacerItem *horizontalSpacer_2;
    QLCDNumber *posz_lcd;
    QSpacerItem *verticalSpacer;
    QLabel *bottom_camera_label;

    void setupUi(QWidget *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName(QString::fromUtf8("guiDlg"));
        guiDlg->resize(769, 549);
        verticalLayout = new QVBoxLayout(guiDlg);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        splitter = new QSplitter(guiDlg);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        beta_frame = new QFrame(splitter);
        beta_frame->setObjectName(QString::fromUtf8("beta_frame"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(beta_frame->sizePolicy().hasHeightForWidth());
        beta_frame->setSizePolicy(sizePolicy);
        beta_frame->setMinimumSize(QSize(400, 0));
        beta_frame->setFrameShape(QFrame::StyledPanel);
        beta_frame->setFrameShadow(QFrame::Raised);
        splitter->addWidget(beta_frame);
        control_frame = new QFrame(splitter);
        control_frame->setObjectName(QString::fromUtf8("control_frame"));
        sizePolicy.setHeightForWidth(control_frame->sizePolicy().hasHeightForWidth());
        control_frame->setSizePolicy(sizePolicy);
        control_frame->setMinimumSize(QSize(200, 0));
        control_frame->setFrameShape(QFrame::StyledPanel);
        control_frame->setFrameShadow(QFrame::Raised);
        verticalLayout_3 = new QVBoxLayout(control_frame);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        top_camera_label = new QLabel(control_frame);
        top_camera_label->setObjectName(QString::fromUtf8("top_camera_label"));
        top_camera_label->setMinimumSize(QSize(0, 200));

        verticalLayout_2->addWidget(top_camera_label);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        speed_lcd = new QLCDNumber(control_frame);
        speed_lcd->setObjectName(QString::fromUtf8("speed_lcd"));
        QFont font;
        font.setBold(false);
        font.setItalic(false);
        font.setWeight(50);
        speed_lcd->setFont(font);
        speed_lcd->setSmallDecimalPoint(false);
        speed_lcd->setMode(QLCDNumber::Dec);
        speed_lcd->setSegmentStyle(QLCDNumber::Flat);

        horizontalLayout->addWidget(speed_lcd);

        speed_label = new QLabel(control_frame);
        speed_label->setObjectName(QString::fromUtf8("speed_label"));

        horizontalLayout->addWidget(speed_label);


        horizontalLayout_2->addLayout(horizontalLayout);


        verticalLayout_2->addLayout(horizontalLayout_2);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        posx_label = new QLabel(control_frame);
        posx_label->setObjectName(QString::fromUtf8("posx_label"));

        horizontalLayout_3->addWidget(posx_label);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer);

        posx_lcd = new QLCDNumber(control_frame);
        posx_lcd->setObjectName(QString::fromUtf8("posx_lcd"));
        posx_lcd->setSegmentStyle(QLCDNumber::Flat);

        horizontalLayout_3->addWidget(posx_lcd);


        horizontalLayout_4->addLayout(horizontalLayout_3);


        verticalLayout_2->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        posz_label = new QLabel(control_frame);
        posz_label->setObjectName(QString::fromUtf8("posz_label"));

        horizontalLayout_5->addWidget(posz_label);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_2);

        posz_lcd = new QLCDNumber(control_frame);
        posz_lcd->setObjectName(QString::fromUtf8("posz_lcd"));
        posz_lcd->setSegmentStyle(QLCDNumber::Flat);

        horizontalLayout_5->addWidget(posz_lcd);


        verticalLayout_2->addLayout(horizontalLayout_5);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        bottom_camera_label = new QLabel(control_frame);
        bottom_camera_label->setObjectName(QString::fromUtf8("bottom_camera_label"));
        bottom_camera_label->setMinimumSize(QSize(0, 200));

        verticalLayout_2->addWidget(bottom_camera_label);


        verticalLayout_3->addLayout(verticalLayout_2);

        splitter->addWidget(control_frame);

        verticalLayout->addWidget(splitter);


        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QWidget *guiDlg)
    {
        guiDlg->setWindowTitle(QApplication::translate("guiDlg", "giraff_viewer", nullptr));
        top_camera_label->setText(QApplication::translate("guiDlg", "TextLabel", nullptr));
        speed_label->setText(QApplication::translate("guiDlg", "speed", nullptr));
        posx_label->setText(QApplication::translate("guiDlg", "position (x)", nullptr));
        posz_label->setText(QApplication::translate("guiDlg", "position (z)", nullptr));
        bottom_camera_label->setText(QApplication::translate("guiDlg", "TextLabel", nullptr));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
