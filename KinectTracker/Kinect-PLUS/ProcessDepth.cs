using System;

namespace KinectTracker
{
    public static class DepthProcessor
    {
        public static void Process(short[] depthData, byte[] depthPixels, ViewerWindow viewer)
        {
            for (int i = 0; i < Constants.IMG_WIDTH * Constants.IMG_HEIGHT; i++)
            {
                short pixel = depthData[i];
                int depthInMm = (pixel & 0xFFFF) >> 3;

                byte intensity;
                if (depthInMm < Constants.MIN_DEPTH || depthInMm > Constants.MAX_DEPTH)
                {
                    intensity = 0;
                }
                else
                {
                    double normalized = 1.0 - ((double)(depthInMm - Constants.MIN_DEPTH) / (Constants.MAX_DEPTH - Constants.MIN_DEPTH));
                    intensity = (byte)(normalized * 255);
                }

                depthPixels[i * 4] = intensity;
                depthPixels[i * 4 + 1] = intensity;
                depthPixels[i * 4 + 2] = intensity;
                depthPixels[i * 4 + 3] = 255;
            }

            viewer.UpdateDepthImage(depthPixels);
        }
    }
}