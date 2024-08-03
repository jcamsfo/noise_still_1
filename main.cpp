#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>

extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
    #include <libavutil/imgutils.h>
}

// Initialize FFmpeg context for decoding an image
void initFFmpegContext(const std::string& imageFile, AVFormatContext*& pFormatCtx, AVCodecContext*& pCodecCtx, int& videoStream) {
    pFormatCtx = avformat_alloc_context();
    if (avformat_open_input(&pFormatCtx, imageFile.c_str(), NULL, NULL) != 0) {
        std::cerr << "Couldn't open file " << imageFile << ".\n";
        exit(-1);
    }

    if (avformat_find_stream_info(pFormatCtx, NULL) < 0) {
        std::cerr << "Couldn't find stream information for " << imageFile << ".\n";
        exit(-1);
    }

    videoStream = -1;
    for (unsigned i = 0; i < pFormatCtx->nb_streams; i++) {
        if (pFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            videoStream = i;
            break;
        }
    }

    if (videoStream == -1) {
        std::cerr << "Didn't find a video stream in " << imageFile << ".\n";
        exit(-1);
    }

    AVCodecParameters *pCodecPar = pFormatCtx->streams[videoStream]->codecpar;
    const AVCodec *pCodec = avcodec_find_decoder(pCodecPar->codec_id);
    if (pCodec == NULL) {
        std::cerr << "Unsupported codec in " << imageFile << "!\n";
        exit(-1);
    }

    pCodecCtx = avcodec_alloc_context3(pCodec);
    if (avcodec_parameters_to_context(pCodecCtx, pCodecPar) < 0) {
        std::cerr << "Couldn't copy codec context for " << imageFile << ".\n";
        exit(-1);
    }

    if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {
        std::cerr << "Couldn't open codec for " << imageFile << ".\n";
        exit(-1);
    }
}

// Load a frame from the image
bool loadImageFrame(AVFormatContext* pFormatCtx, AVCodecContext* pCodecCtx, int videoStream, cv::Mat& img) {
    AVFrame *pFrame = av_frame_alloc();
    AVFrame *pFrameGray = av_frame_alloc();
    if (!pFrameGray) {
        exit(-1);
    }

    uint8_t *buffer = (uint8_t *)av_malloc(av_image_get_buffer_size(AV_PIX_FMT_GRAY8, pCodecCtx->width, pCodecCtx->height, 1));
    av_image_fill_arrays(pFrameGray->data, pFrameGray->linesize, buffer, AV_PIX_FMT_GRAY8, pCodecCtx->width, pCodecCtx->height, 1);

    struct SwsContext *sws_ctx = sws_getContext(pCodecCtx->width,
                                                pCodecCtx->height,
                                                pCodecCtx->pix_fmt,
                                                pCodecCtx->width,
                                                pCodecCtx->height,
                                                AV_PIX_FMT_GRAY8,
                                                SWS_BILINEAR,
                                                NULL, NULL, NULL);

    AVPacket packet;
    bool frameLoaded = false;

    while (av_read_frame(pFormatCtx, &packet) >= 0) {
        if (packet.stream_index == videoStream) {
            avcodec_send_packet(pCodecCtx, &packet);
            if (avcodec_receive_frame(pCodecCtx, pFrame) == 0) {
                sws_scale(sws_ctx, (uint8_t const * const *)pFrame->data,
                          pFrame->linesize, 0, pCodecCtx->height,
                          pFrameGray->data, pFrameGray->linesize);

                img = cv::Mat(pCodecCtx->height, pCodecCtx->width, CV_8UC1, pFrameGray->data[0], pFrameGray->linesize[0]).clone();
                frameLoaded = true;
                break;
            }
        }
        av_packet_unref(&packet);
    }

    if (!frameLoaded) {
        std::cerr << "Failed to load frame from image.\n";
    }

    sws_freeContext(sws_ctx);
    av_free(buffer);
    av_frame_free(&pFrameGray);
    av_frame_free(&pFrame);

    return frameLoaded;
}

// Generate grayscale noise frames
std::vector<cv::Mat> generateNoiseFrames(int width, int height, int numFrames) {
    std::vector<cv::Mat> noiseFrames;
    for (int i = 0; i < numFrames; ++i) {
        cv::Mat noise(height, width, CV_8UC1);
        cv::randu(noise, cv::Scalar(0), cv::Scalar(255));
        noiseFrames.push_back(noise);
    }
    return noiseFrames;
}

int main() {

    std::string imageFile = "/home/jim/Desktop/PiTests/images/image.jpg"; // Path to your image file

    AVFormatContext *pFormatCtx;
    AVCodecContext *pCodecCtx;
    int videoStream;

    initFFmpegContext(imageFile, pFormatCtx, pCodecCtx, videoStream);

    cv::Mat img, blendedImg;
    if (!loadImageFrame(pFormatCtx, pCodecCtx, videoStream, img)) {
        std::cerr << "Failed to load image frame.\n";
        return -1;
    }

    const int numNoiseFrames = 30;
    std::vector<cv::Mat> noiseFrames = generateNoiseFrames(img.cols, img.rows, numNoiseFrames);

    cv::namedWindow("Blended Image Playback", cv::WINDOW_AUTOSIZE);
    int noiseFrameIndex = 0;

    while (true) {
        auto loopStartTime = std::chrono::steady_clock::now();

        // Use pre-generated noise frame
        cv::Mat noise = noiseFrames[noiseFrameIndex];
        noiseFrameIndex = (noiseFrameIndex + 1) % numNoiseFrames;

        // Blend image with noise (50% opacity)
        cv::addWeighted(img, 0.5, noise, 0.5, 0.0, blendedImg);
        cv::imshow("Blended Image Playback", blendedImg);

        if (cv::waitKey(32) >= 0) {
            break;
        }

        auto loopEndTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = loopEndTime - loopStartTime;
        std::cout << "Loop duration: " << elapsed_seconds.count() << "s\n";
    }

    avcodec_close(pCodecCtx);
    avformat_close_input(&pFormatCtx);

    return 0;
}
