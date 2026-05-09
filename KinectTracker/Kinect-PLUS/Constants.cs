namespace KinectTracker
{
    public static class Constants
    {
        public const int IMG_WIDTH = 640;
        public const int IMG_HEIGHT = 480;

        //Valor RGB para solo ver las esferas
        public const int THRESHOLD = 230;

        public const int MIN_DEPTH = 800;   //mm
        public const int MAX_DEPTH = 4000;  //mm

        //pixeles para buscar depth válido alrededor del centroide
        public const int SEARCH_RADIUS = 30;

        //Parámetros de detección de blobs
        public const int MIN_BLOB_AREA = 0; //Píxeles mínimos, 0 para aumentar sensibilidad a larga distancia (+1m)
        public const int MAX_BLOB_AREA = 1000;
        public const double MIN_CIRCULARITY = 0.5; //Esferas son redondas (~1.0), rechaza líneas/ruido
        public const double MIN_ASPECT = 0.6; //Aspect ratio mínimo (rechaza líneas alargadas, como herramientas)
        public const double MAX_ASPECT = 1.7;
    }
}
