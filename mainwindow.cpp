#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "AVL.h"
#include <vector>
#include <QPixmap>
#include "math.h"
using namespace std;


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
    avl::Image image1,image_hsl,dark_image,image_sum,sum_dark,filtered_image,CannyEdges;
    avl::LoadImage("road lines1.jpg",false,image1);
    avl::Image image2,channel1,channel2,channel3;
    avl::AverageChannels(image1,atl::NIL,image2);

    avl::RgbToHsl(image1,image_hsl);
    
    avl::SaveImage(image_hsl,atl::NIL,"save_hsl.jpg",0);
    

    avl::Image yellow,white;
    //yellow
    avl::ThresholdImage_HSx(image1,atl::NIL,avl::HSxColorModel::Type::HSL,0,255,160,255,140,255,2,yellow,channel2);
    //avl::RegionToImage(region1,yellow_mask);
    //white
    avl::ThresholdImage_HSx(image1,atl::NIL,avl::HSxColorModel::Type::HSL,0,255,0,210,190,255,2,white,channel3);
    //avl::RegionToImage(region2,white_mask);

    avl::SaveImage(yellow,atl::NIL,"yellow_mask.jpg",0);
    avl::SaveImage(white,atl::NIL,"white_mask.jpg",0);

    //avl::SaveImage(channel2,atl::NIL,"ch2.jpg",0);
    avl::AddImages(yellow,white,atl::NIL,1.0f,image_sum);

    avl::EqualizeImageHistogram(image2,atl::NIL,0.0f,0.5f,dark_image);

    avl::SaveImage(dark_image,atl::NIL,"save.jpg",0);
    avl::SaveImage(image_sum,atl::NIL,"image_sum.jpg",0);
    avl::MultiplyImages(dark_image,image_sum,atl::NIL,1.0f,sum_dark);
    avl::SaveImage(sum_dark,atl::NIL,"sum_dark.jpg",0);
    avl::SmoothImage_Gauss(sum_dark,atl::NIL,1.0f,atl::NIL,3.0f,filtered_image);
    avl::SaveImage(filtered_image,atl::NIL,"filtered_image.jpg",0);

    //avl::OpenImage(filtered_image,atl::NIL,atl::NIL,atl::NIL,avl::KernelShape::Box,7,4,filtered_image);
    avl::DetectEdges(filtered_image,atl::NIL,avl::EdgeFilter::Type::Canny,2.0f,atl::NIL,15.0f,5.0f,0.0f,1,CannyEdges,channel3);

    avl::SaveImage(CannyEdges,atl::NIL,"canny.jpg",0);

    int H = CannyEdges.Height();
    int W = CannyEdges.Width();
    avl::Circle2D c(W/2,2*H,1.4*H);

    //Rectangle2D r({0,H/2},0,W,H/2);
    avl::Rectangle2D r({W*0.2f,17*H/32},0,W*0.7f,H/2);
    avl::Region region2,regionc;
    avl::CreateRectangleRegion(r,atl::NIL,W,H,region2,atl::NIL);
    avl::CreateCircleRegion(c,atl::NIL,W,H,regionc,atl::NIL);
    avl::Image cut;
    avl::CopyImageData(CannyEdges,regionc,cut);
    avl::SaveImage(cut,atl::NIL,"cut.jpg",0);

    atl::Array <avl::Line2D> outLines;
    atl::Array <float> outScores;
    avl::DetectLines(cut,atl::NIL,1.0f,20.0f,40.0f,15.0f,20.0f,outLines,outScores,channel1,channel2);

    avl::Pixel pix;
    avs::AvsFilter_MakePixel(255.0f,1.0f,0.0f,255.0f,pix);
    avl::DrawingStyle DS;
    DS.opacity=1.0f;
    DS.filled=true;
    DS.thickness=5;

    float a = tan(135),b = tan(45);
    for (const auto &i:outLines){
        float k=i.A()/-i.B();
        if (tanh(k)>135 && tanh(k)<45)
            avl::DrawLine(cut,i,atl::NIL,pix,DS);
    }
//    avl::Image test;
//    avl::LoadImage("zebra.jpg",false,test);
//    avl::DetectLines(test,atl::NIL,1.0f,20.0f,40.0f,20.0f,10.0f,outLines,outScores,channel1,channel2);
//    for (const auto &i:outLines){
//            avl::DrawLine(test,i,atl::NIL,pix,DS);
//    }
//    avl::SaveImage(test,atl::NIL,"zzebra.jpg",0);

    avl::SaveImage(cut,atl::NIL,"x.jpg",0);

    QPixmap mp("canny.jpg");

    ui->label->setPixmap(mp);

    ui->label->setScaledContents(true);
}

void MainWindow::on_pushButton_2_clicked()
{
   this->close();
}
