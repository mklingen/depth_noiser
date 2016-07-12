#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <random>

sensor_msgs::ImageConstPtr sensorImg;
bool newData;

 std::default_random_engine generator;
 std::normal_distribution<double> distribution(0, 1);

void sensorCallback(const sensor_msgs::ImageConstPtr& img)
{
    sensorImg = img;
    newData = true;
}


template <typename T> bool noiseImages(sensor_msgs::ImageConstPtr& sensor, sensor_msgs::ImagePtr imgOut, float noiseConstant, float noiseLinear, float noiseQuadratic)
{
    if (!sensor.get())
    {
        ROS_ERROR("Images were not received.");
        return false;
    }

    imgOut->width = sensor->width;
    imgOut->height = sensor->height;
    imgOut->step = sensor->step;
    imgOut->is_bigendian = sensor->is_bigendian;
    imgOut->header = sensor->header;
    imgOut->data = sensor->data;
    imgOut->encoding = sensor->encoding;


    const T* sensorData = reinterpret_cast<const T*>(sensor->data.data());
    T* imgData = reinterpret_cast<T*>(imgOut->data.data());

    for (size_t i = 0; i < sensor->width * sensor->height; i++)
    {
        const T& pixel = sensorData[i];
        T noiseSigma = static_cast<T>(noiseConstant + noiseLinear * pixel + noiseQuadratic * (pixel * pixel));
        imgData[i] = static_cast<T>(imgData[i] + distribution(generator) * noiseSigma);
    }
    return true;
}

int main(int argc, char** argv)
{
    newData = false;
    ros::init(argc, argv, "depth_mask");
    ros::NodeHandle nh("~");

    ros::Subscriber sensorSub = nh.subscribe("/depth_image", 10, &sensorCallback);
    ros::Publisher publisher = nh.advertise<sensor_msgs::Image>("/noised_depth", 10);

    ros::Rate sleepRate(60);

    double noiseConstant, noiseLinear, noiseQuadratic;
    nh.param("constant_noise", noiseConstant, 0.001);
    nh.param("linear_noise", noiseLinear, 0.01);
    nh.param("quadratic_noise", noiseQuadratic, 0.001);

    sensor_msgs::ImagePtr img(new sensor_msgs::Image());
    
    while (ros::ok())
    {
        ros::spinOnce();
        sleepRate.sleep();

        if (!newData) continue;

        if (sensorImg.get())
        {
            if (sensorImg->encoding == "16UC1")
            {
                if(!noiseImages<uint16_t>(sensorImg, img, (float)(noiseConstant * 1000), (float)(noiseLinear * 1000), (float)(noiseQuadratic * 1000)))
                {
                    return -1;
                }
            }
            else if (sensorImg->encoding == "32FC1")
            {
                if(!noiseImages<float>(sensorImg, img, (float)(noiseConstant * 1000), (float)(noiseLinear * 1000), (float)(noiseQuadratic * 1000)))
                {
                    return -1;
                }
            }
            else
            {
                ROS_ERROR("Can't process image of type %s. Need 16UC1 or 32FC1", sensorImg->encoding.c_str());
                return -1;
            }
            publisher.publish(img);
            newData = false;
        }
    }

    return 0;
}
