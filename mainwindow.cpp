#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "AVL.h"
#include <vector>
#include <QPixmap>
#include "math.h"
#include <string>
#define PI 3.14159265

void Save(avl::Image image, const atl::File& inFile){
    bool IgnoreErrors=0;
    avl::SaveImage(image, avl::ImageFileFormat::Type::JPEG,inFile, IgnoreErrors);
}

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
    avl::Image image1,dark_image,image_sum,sum_dark,filtered_image,CannyEdges,image2,channel1,channel2,channel3;;
    //Load test image
    bool InLoadAlphaChannel=false;
    avl::LoadImage("road lines1.jpg",InLoadAlphaChannel,image1);
    //Region of Interest= whole image
    atl::Optional<const avl::Region&> inRoi=atl::NIL;
    //RGB -> black & white
    avl::AverageChannels(image1,inRoi,image2);
    avl::Image yellow,white;
    //select yellow mask
    int BeginHue=0, EndHue=255, MinSaturation=160,MaxSaturation=255, MinBrightness=140, MaxBrightness=255,inFuzziness=2;
    avl::ThresholdImage_HSx(image1,inRoi,avl::HSxColorModel::Type::HSL, BeginHue, EndHue, MinSaturation, MaxSaturation, MinBrightness, MaxBrightness,inFuzziness,yellow);
    //select white mask
    BeginHue=0, EndHue=255, MinSaturation=0, MaxSaturation=210, MinBrightness=190, MaxBrightness=255,inFuzziness=2;
    avl::ThresholdImage_HSx(image1,inRoi,avl::HSxColorModel::Type::HSL,BeginHue, EndHue, MinSaturation, MaxSaturation, MinBrightness, MaxBrightness,inFuzziness, white);

    Save(yellow,"yellow_mask");
    Save(white,"white_mask");
    //sum masks
    float inScale=1.0f;
    avl::AddImages(yellow,white,inRoi,inScale,image_sum);
    //image->dark image
    float SaturateBrightestFraction=0.0f;
    float SaturateDarkestFraction=0.5f;
    avl::EqualizeImageHistogram(image2,inRoi,SaturateBrightestFraction,SaturateDarkestFraction,dark_image);

    Save(dark_image,"dark_image");
    Save(image_sum,"image_sum.jpg");
    //dark image* yellow&white mask
    avl::MultiplyImages(dark_image,image_sum,inRoi,1.0f,sum_dark);
    Save(sum_dark,"sum_dark.jpg");
    //gauss filter with 3x3 kernel
    float horizont_smooth=1.0f;
    float vertical_smooth=atl::NIL;
    float kernelSize=3.0f;
    avl::SmoothImage_Gauss(sum_dark,inRoi, horizont_smooth,vertical_smooth,kernelSize,filtered_image);
    Save(filtered_image,"filtered_image.jpg");
    //CannyDetection
    float horizontal_smooth_edge_filter=2.0f;
    float vertical_smooth_edge_filter=atl::NIL;
    float inEdgeThreshold=15.0f, inEdgeHysteresis=5.0f, inMaxJoiningDistance=0.0f;
    int inMinBlobArea=1;
    avl::DetectEdges(filtered_image,inRoi,avl::EdgeFilter::Type::Canny, horizontal_smooth_edge_filter, vertical_smooth_edge_filter, inEdgeThreshold, inEdgeHysteresis, inMaxJoiningDistance, inMinBlobArea,CannyEdges);
    Save(CannyEdges,"canny");
    //Select circle region
    int H = CannyEdges.Height();
    int W = CannyEdges.Width();
    avl::Circle2D Circle(W/2, 1.2*H, 0.6*H);
    avl::Region regionc;
    avl::CreateCircleRegion(Circle, atl::NIL, W, H, regionc, atl::NIL);
    avl::Image cut;
    avl::CopyImageData(CannyEdges,regionc,cut);
    Save(cut,"cut.jpg");
    //Lines detection
    atl::Array <avl::Line2D> outLines;
    atl::Array <float> outScores;
    float inAngleResolution=1.0f, inMinAngleDelta=20.0f,inMinDistance=100.0f,inMinScore=15.0f, EdgeThreshold=25.0f;
    avl::DetectLines(cut, regionc, inAngleResolution, inMinAngleDelta, inMinDistance, inMinScore, EdgeThreshold, outLines,outScores);
    //Drawing
    avl::Pixel pix;
    float inX=255.0f;
    float inY=0.0f;
    float inZ=0.0f;
    float inW=255.0f;
    avs::AvsFilter_MakePixel(inX, inY, inZ, inW, pix);

    avl::DrawingStyle DS;
    DS.opacity=1.0f;
    DS.filled=true;
    DS.thickness=25;
    float k1=0.95, k2=1.3;
    for (const auto &i:outLines){
        if ((abs(i.a/i.b) > k1) && (abs(i.a/i.b) < k2) )
            avl::DrawLine(image1,i,atl::NIL,pix,DS);
    }
    Save(cut,"x.jpg");
    Save(image1,"result.jpg");


    //show result in label
    QPixmap mp("result.jpg");

    ui->label->setPixmap(mp);

    ui->label->setScaledContents(true);
}

void MainWindow::on_pushButton_2_clicked()
{
    //close
   this->close();
}

void MainWindow::on_pushButton_3_clicked()
{
    //load test image
    QPixmap mp("road lines1.jpg");

    ui->label->setPixmap(mp);

    ui->label->setScaledContents(true);
}
