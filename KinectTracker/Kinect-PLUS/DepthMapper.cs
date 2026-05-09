using Microsoft.Kinect;

namespace KinectTracker
{
    public class DepthMapper
    {
        private KinectSensor sensor;

        public DepthMapper(KinectSensor sensor) //Constructor
        {
            this.sensor = sensor;
        }


        public int FindValidDepth(int x, int y, short[] depthData)
        {
            //Busca Depth válido alrededor del centroide (x,y) en un radio definido buscando el MÍNIMO válido
            //(las esferas reflectivas saturan el sensor, pero el borde sí suele dar depth válido)
            int depthMm = int.MaxValue;

            for (int dy = -Constants.SEARCH_RADIUS; dy <= Constants.SEARCH_RADIUS; dy++)
            {
                for (int dx = -Constants.SEARCH_RADIUS; dx <= Constants.SEARCH_RADIUS; dx++)
                {
                    int sx = x + dx;
                    int sy = y + dy;
                    if (sx < 0 || sx >= Constants.IMG_WIDTH || sy < 0 || sy >= Constants.IMG_HEIGHT) continue;

                    int sIdx = sy * Constants.IMG_WIDTH + sx;
                    int rawSample = depthData[sIdx] & 0xFFFF; //Forzar interpretación sin signo
                    int sample = rawSample >> 3;

                    if (sample >= Constants.MIN_DEPTH && sample <= Constants.MAX_DEPTH && sample < depthMm)
                    {
                        depthMm = sample;
                    }
                }
            }

            if (depthMm == int.MaxValue)
                return -1;
            //Validar depth
            //if (depthMm == int.MaxValue || depthMm < Constants.MIN_DEPTH || depthMm > Constants.MAX_DEPTH)
              //  continue;

            return depthMm;
        }

        //Convierte (x, y) en pixeles + depth en mm a coordenadas 3D del mundo
        public SkeletonPoint ConvertTo3D(int x, int y, int depthMm) {
            return sensor.CoordinateMapper.MapDepthPointToSkeletonPoint(
                DepthImageFormat.Resolution640x480Fps30,
                new DepthImagePoint { X = x, Y = y, Depth = depthMm }
            );

        }

    }
}
