#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "AVL.h"
#include <vector>
#include <QPixmap>
#include "math.h"
#define PI 3.14159265


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_clicked()
{
    avl::Image image1,dark_image,image_sum,sum_dark,filtered_image,CannyEdges;
    //Load test image
    avl::LoadImage("road lines1.jpg",false,image1);
    avl::Image image2,channel1,channel2,channel3;
    avl::AverageChannels(image1,atl::NIL,image2);
    avl::Image yellow,white;
    //select yellow mask
    avl::ThresholdImage_HSx(image1,atl::NIL,avl::HSxColorModel::Type::HSL,0,255,160,255,140,255,2,yellow,channel2);
    //select white mask
    avl::ThresholdImage_HSx(image1,atl::NIL,avl::HSxColorModel::Type::HSL,0,255,0,210,190,255,2,white,channel3);

    avl::SaveImage(yellow,atl::NIL,"yellow_mask.jpg",0);
    avl::SaveImage(white,atl::NIL,"white_mask.jpg",0);
    //sum masks
    avl::AddImages(yellow,white,atl::NIL,1.0f,image_sum);
    //image->dark image
    avl::EqualizeImageHistogram(image2,atl::NIL,0.0f,0.5f,dark_image);

    avl::SaveImage(dark_image,atl::NIL,"save.jpg",0);
    avl::SaveImage(image_sum,atl::NIL,"image_sum.jpg",0);
    //dark image* yellow&white mask
    avl::MultiplyImages(dark_image,image_sum,atl::NIL,1.0f,sum_dark);
    avl::SaveImage(sum_dark,atl::NIL,"sum_dark.jpg",0);
    //gauss filter with 3x3 kernel
    avl::SmoothImage_Gauss(sum_dark,atl::NIL,1.0f,atl::NIL,3.0f,filtered_image);
    avl::SaveImage(filtered_image,atl::NIL,"filtered_image.jpg",0);
    //CannyDetection
    avl::DetectEdges(filtered_image,atl::NIL,avl::EdgeFilter::Type::Canny,2.0f,atl::NIL,15.0f,5.0f,0.0f,1,CannyEdges,channel3);
    avl::SaveImage(CannyEdges,atl::NIL,"canny.jpg",0);
    //Select circle region
    int H = CannyEdges.Height();
    int W = CannyEdges.Width();
    avl::Circle2D c(W/2,1.2*H,0.6*H);
    avl::Region regionc;
    avl::CreateCircleRegion(c,atl::NIL,W,H,regionc,atl::NIL);
    avl::Image cut;
    avl::CopyImageData(CannyEdges,regionc,cut);
    avl::SaveImage(cut,atl::NIL,"cut.jpg",0);
    //Lines detection
    atl::Array <avl::Line2D> outLines;
    atl::Array <float> outScores;
    avl::DetectLines(cut,regionc,1.0f,20.0f,80.0f,10.0f,25.0f,outLines,outScores,channel1,channel2);
    //Drawing
    avl::Pixel pix;
    avs::AvsFilter_MakePixel(255.0f,1.0f,0.0f,255.0f,pix);
    avl::DrawingStyle DS;
    DS.opacity=1.0f;
    DS.filled=true;
    DS.thickness=25;
    for (const auto &i:outLines){
        if ((abs(i.a/i.b) > 0.9) && (abs(i.a/i.b) < 1.3) )
            avl::DrawLine(image1,i,atl::NIL,pix,DS);
    }
    avl::SaveImage(cut,atl::NIL,"x.jpg",0);
    avl::SaveImage(image1,atl::NIL,"result.jpg",0);

    //Output to labeb
    QPixmap mp("result.jpg");

    ui->label->setPixmap(mp);

    ui->label->setScaledContents(true);
}

void MainWindow::on_pushButton_2_clicked()
{
   this->close();
}
