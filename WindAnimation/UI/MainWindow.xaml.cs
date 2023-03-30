using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.IO;
using WindVizLib;

namespace WindAnimation
{
  /// <summary>
  /// Interaction logic for MainWindow.xaml
  /// </summary>
  public partial class MainWindow : Window, INotifyPropertyChanged
  {
    public event PropertyChangedEventHandler PropertyChanged;
    private BitmapSource m_image;
    private WindParticleEmitter m_emitter;
    public string WorkingDir => Path.GetDirectoryName( Application.ResourceAssembly.Location);
    public BitmapSource ImageSource => m_image;
    public BitmapImage BlueMarble => new BitmapImage(new Uri(Path.Combine(WorkingDir, "world.topo.bathy.200411.3x5400x2700.jpg")));
    public int Width => 1080 * 2;
    public int Height => 540 * 2;

    public MainWindow()
    {
      InitializeComponent();
      this.DataContext = this;
      //m_emitter.Initialize(23, 0.6);
      m_image = new BitmapImage(new Uri(Path.Combine(WorkingDir, "Wind.png")));
      int rawStride = (m_image.Format.BitsPerPixel + 7) / 8;
      int stride = (int)m_image.PixelWidth * rawStride;
      byte[] pixels = new byte[(int)m_image.PixelHeight * stride];
      m_image.CopyPixels(pixels, stride, 0);
      m_emitter = new WindParticleEmitter(pixels, m_image.PixelWidth, m_image.PixelHeight, 4);
      m_emitter.Initialize(30000, 50);
      Task.Run(UpdateImage);
    }

    protected void OnPropertyChanged([CallerMemberName] string name = null)
    {
      PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
    }

    private void UpdateImage()
    {
      while(true)
      {
        Application.Current?.Dispatcher.Invoke(new Action(() =>
        {
          //m_image = new BitmapImage(new Uri("C:/Code/" + m_imageFiles[m_flag]));

          // Define parameters used to create the BitmapSource.
          PixelFormat pf = PixelFormats.Bgra32;
          int width = Width;
          int height = Height;
          int rawStride = (width * pf.BitsPerPixel + 7) / 8;
          byte[] rawImage = new byte[rawStride * height];
          m_emitter.Update();
          m_emitter.Render(width, height, ref rawImage);
          // Initialize the image with data.
          //Random value = new Random();
          //value.NextBytes(rawImage);

          // Create a BitmapSource.
          m_image = BitmapSource.Create(width, height,
              96, 96, pf, null,
              rawImage, rawStride);

          OnPropertyChanged(nameof(ImageSource));
        }));
        Thread.Sleep(10);
      }
    }
  }
}
