/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   Copyright 2021 Kenneth W. Chapman

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "guivisapp.h"
#include <QMenu>
#include <QAction>
#include <QComboBox>
#include <QContextMenuEvent>
#include <QtWidgets/QLabel>
#include <QtWidgets/QRubberBand>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QProgressDialog>
#include <QtWidgets/QListWidget>

#ifdef QT_NO_CONTEXTMENU
#undef QT_NO_CONTEXTMENU
#endif

Q_DECLARE_METATYPE( std::string );

using namespace gc;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
#ifndef QT_NO_CONTEXTMENU
    void contextMenuEvent( QContextMenuEvent * ) override;
#endif // QT_NO_CONTEXTMENU

signals:
    void sig_visAppMessage( const QString imgPath );
    void sig_updateProgess( const int );
    void sig_visAppLineFound( const QString imgPath );

private slots:
    void on_actionZoomToFit_triggered();
    void on_actionZoom100_triggered();
    void on_horizontalSlider_zoom_valueChanged( int );
    void on_actionExit_triggered();
    void on_actionImageLoad_triggered();
    void on_actionImageSave_triggered();
    void on_actionSetROI_toggled( bool );
    void on_actionSetRuler_triggered();
    void on_lineEdit_imageFolder_textEdited( const QString &strPath );
    void on_listWidget_imageFolder_currentRowChanged(int row);
    void on_toolButton_imageFolder_browse_clicked();
    void on_toolButton_clearMsgs_clicked();
    void on_visAppMessage( const string msg );
    void do_visAppMessage( const QString msg );
    void on_updateProgress( const int );
    void do_updateProgress( const int );
    void on_tableAddRow( const string row_strings );
    void on_actionSaveVideo_triggered();
    void on_pushButton_visionCalibrate_clicked();
    void on_toolButton_calibVisionTarget_csv_browse_clicked();
    void on_toolButton_calibVisionResult_json_browse_clicked();
    void on_toolButton_findLineTopFolder_browse_clicked();
    void on_toolButton_findLine_resultCSVFile_browse_clicked();
    void on_toolButton_findLine_annotatedResultFolder_browse_clicked();
    void on_pushButton_findLineCurrentImage_clicked();
    void on_pushButton_findLine_processFolder_clicked();
    void on_pushButton_findLine_stopFolderProcess_clicked();
    void on_pushButton_showImageMetadata_clicked();

    void on_pushButton_test_clicked();

private:
    Ui::MainWindow *ui;

    QString m_folderLoadImages;
    QString m_folderSaveImages;
    QString m_textEditBuffer;

    // image settings
    int m_imgWidth;
    int m_imgHeight;

    // flags
    bool m_bCaptured;
    bool m_bFinishingFolderThread;
    int m_nCapturePos;

    // QT display stuff
    QComboBox *m_pComboBoxImageToView;
    QImage *m_pQImg;
    QLabel *m_pLabelImgDisplay;
    double m_scaleFactor;

    // region of interest code
    QLine m_lineOne;
    QRubberBand *m_pRubberBand;
    QRect m_rectROI;
    QRect m_rectRubberBand;
    QPoint m_ptCapture;

    // vision application
    GuiVisApp m_visApp;
    std::vector< std::string > m_imageFilePaths;

    // paint, mouse, and application event methods
    void mouseReleaseEvent( QMouseEvent * ) override;
    void mouseDoubleClickEvent( QMouseEvent * ) override;
    void paintEvent( QPaintEvent * ) override;
    void mousePressEvent( QMouseEvent *pEvent ) override;
    void mouseMoveEvent( QMouseEvent *pEvent ) override;
    void ScaleImage();
    void ZoomTo( const int width, const int height );

    // context menu
    QAction *setRuler_inView;
    void createActions();
    void createConnections();
    QAction *actionRoiAdd;

    // helper methods
    void UpdateGUIEnables();
    void UpdatePixmap();
    void UpdatePixmapTarget();
    void TestAgainstFindLines( QPoint pt );
    void TestAgainstRubberBands( QPoint pt );
    void AdjustPointRubberBand();
    void AdjustPointFindLines();
    int ReadSettings( const QString filepath = "" );
    int WriteSettings(const QString filepath = "" );
    size_t GetImagesPathsFromFolder(  const QString strPath );
    int ResizeImage( const int nWidth, const int nHeight );
    IMG_BUFFERS GetImageColor( const QString image_color );
    void ClearTable();
    int InitTable( const vector< string > headings );
    int AddRow( const string row_string );
};

#endif // MAINWINDOW_H
