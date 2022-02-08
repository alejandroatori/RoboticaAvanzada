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
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollBar>
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
    QLCDNumber *tilt_lcdnumber;
    QLabel *label;
    QScrollBar *tilt_scrollbar;
    QHBoxLayout *horizontalLayout_4;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *sweep_button;
    QSpacerItem *horizontalSpacer;
    QLCDNumber *sweep_lcdNumber;
    QLabel *label_2;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *trace_button;
    QSpacerItem *horizontalSpacer_2;
    QSpacerItem *verticalSpacer;

    void setupUi(QWidget *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName(QString::fromUtf8("guiDlg"));
        guiDlg->resize(769, 443);
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
        tilt_lcdnumber = new QLCDNumber(control_frame);
        tilt_lcdnumber->setObjectName(QString::fromUtf8("tilt_lcdnumber"));

        horizontalLayout->addWidget(tilt_lcdnumber);

        label = new QLabel(control_frame);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);


        horizontalLayout_2->addLayout(horizontalLayout);

        tilt_scrollbar = new QScrollBar(control_frame);
        tilt_scrollbar->setObjectName(QString::fromUtf8("tilt_scrollbar"));
        tilt_scrollbar->setMinimum(-10);
        tilt_scrollbar->setMaximum(70);
        tilt_scrollbar->setOrientation(Qt::Horizontal);

        horizontalLayout_2->addWidget(tilt_scrollbar);


        verticalLayout_2->addLayout(horizontalLayout_2);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        sweep_button = new QPushButton(control_frame);
        sweep_button->setObjectName(QString::fromUtf8("sweep_button"));
        sweep_button->setCheckable(true);

        horizontalLayout_3->addWidget(sweep_button);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer);

        sweep_lcdNumber = new QLCDNumber(control_frame);
        sweep_lcdNumber->setObjectName(QString::fromUtf8("sweep_lcdNumber"));

        horizontalLayout_3->addWidget(sweep_lcdNumber);

        label_2 = new QLabel(control_frame);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_3->addWidget(label_2);


        horizontalLayout_4->addLayout(horizontalLayout_3);


        verticalLayout_2->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        trace_button = new QPushButton(control_frame);
        trace_button->setObjectName(QString::fromUtf8("trace_button"));
        trace_button->setCheckable(true);

        horizontalLayout_5->addWidget(trace_button);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_2);


        verticalLayout_2->addLayout(horizontalLayout_5);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);


        verticalLayout_3->addLayout(verticalLayout_2);

        splitter->addWidget(control_frame);

        verticalLayout->addWidget(splitter);


        retranslateUi(guiDlg);
        QObject::connect(tilt_scrollbar, SIGNAL(valueChanged(int)), tilt_lcdnumber, SLOT(display(int)));

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QWidget *guiDlg)
    {
        guiDlg->setWindowTitle(QApplication::translate("guiDlg", "giraff_viewer", nullptr));
        top_camera_label->setText(QApplication::translate("guiDlg", "TextLabel", nullptr));
        label->setText(QApplication::translate("guiDlg", "tablet tilt", nullptr));
        sweep_button->setText(QApplication::translate("guiDlg", "sweep", nullptr));
        label_2->setText(QApplication::translate("guiDlg", "%", nullptr));
        trace_button->setText(QApplication::translate("guiDlg", "trace", nullptr));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
