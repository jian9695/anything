using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace ScrollVeiwerTouch
{
  /// <summary>
  /// Interaction logic for MainWindow.xaml
  /// </summary>
  public partial class MainWindow : Window
  {
    public class DocumentmentNode
    {
      public List<FlowDocument> DocItems { get; set; } = new List<FlowDocument>();
      public string Name { get; set; } = "";
    }

    public List<FlowDocument> DocItems { get; set; } = new List<FlowDocument>();
    public List<string> Items { get; set; } = new List<string>();
    public List<DocumentmentNode> DocItemsTree { get; set; } = new List<DocumentmentNode>();

    public FlowDocument Document
    {
      get
      {
        string str = "How can I alter the default behavior of the ScrollViewer class? I tried inheriting from the ScrollViewer class and overriding the MeasureOverride and ArrangeOverride classes, but I couldn't figure out how to measure and arrange the child properly. It appears that the arrange has to affect the ScrollContentPresenter somehow, not the actual content child.How can I alter the default behavior of the ScrollViewer class? I tried inheriting from the ScrollViewer class and overriding the MeasureOverride and ArrangeOverride classes, but I couldn't figure out how to measure and arrange the child properly. It appears that the arrange has to affect the ScrollContentPresenter somehow, not the actual content child.How can I alter the default behavior of the ScrollViewer class? I tried inheriting from the ScrollViewer class and overriding the MeasureOverride and ArrangeOverride classes, but I couldn't figure out how to measure and arrange the child properly. It appears that the arrange has to affect the ScrollContentPresenter somehow, not the actual content child.How can I alter the default behavior of the ScrollViewer class? I tried inheriting from the ScrollViewer class and overriding the MeasureOverride and ArrangeOverride classes, but I couldn't figure out how to measure and arrange the child properly. It appears that the arrange has to affect the ScrollContentPresenter somehow, not the actual content child.";
        FlowDocument doc = new FlowDocument();
        Paragraph p = new Paragraph(new Run(str));
        p.FontSize = 36;
        doc.Blocks.Add(p);
        p.FontSize = 14;
        p.FontStyle = FontStyles.Italic;
        p.TextAlignment = TextAlignment.Left;
        p.Foreground = Brushes.Gray;
        doc.Blocks.Add(p);
        return doc;
      }
    }

    static T FindVisualChild<T>(DependencyObject parent)
    {
      if (typeof(T) == parent.GetType())
        return (T)(object)parent;
      int num = VisualTreeHelper.GetChildrenCount(parent);
      if (num == 0)
        return (T)(object)null;
      for (int i = 0; i < num; i++)
      {
        DependencyObject child = VisualTreeHelper.GetChild(parent, i);
        var result = FindVisualChild<T>(child);
        if (result != null)
          return result;
      }
      return (T)(object)null;
    }

    public MainWindow()
    {
      string str = "How can I alter the default behavior of the ScrollViewer class? I tried inheriting from the ScrollViewer class and overriding the MeasureOverride and ArrangeOverride classes, but I couldn't figure out how to measure and arrange the child properly. It appears that the arrange has to affect the ScrollContentPresenter somehow, not the actual content child.How can I alter the default behavior of the ScrollViewer class? I tried inheriting from the ScrollViewer class and overriding the MeasureOverride and ArrangeOverride classes, but I couldn't figure out how to measure and arrange the child properly. It appears that the arrange has to affect the ScrollContentPresenter somehow, not the actual content child.How can I alter the default behavior of the ScrollViewer class? I tried inheriting from the ScrollViewer class and overriding the MeasureOverride and ArrangeOverride classes, but I couldn't figure out how to measure and arrange the child properly. It appears that the arrange has to affect the ScrollContentPresenter somehow, not the actual content child.How can I alter the default behavior of the ScrollViewer class? I tried inheriting from the ScrollViewer class and overriding the MeasureOverride and ArrangeOverride classes, but I couldn't figure out how to measure and arrange the child properly. It appears that the arrange has to affect the ScrollContentPresenter somehow, not the actual content child.";
      for (int i = 0; i < 10; i++)
      {
        FlowDocument doc = new FlowDocument();
        Paragraph p = new Paragraph(new Run(str));
        p.FontSize = 36;
        doc.Blocks.Add(p);
        p.FontSize = 14;
        p.FontStyle = FontStyles.Italic;
        p.TextAlignment = TextAlignment.Left;
        p.Foreground = Brushes.Gray;
        doc.Blocks.Add(p);

        DocItems.Add(doc);
        Items.Add(str);
      }

      for (int i = 0; i < 5; i++)
      {
        DocumentmentNode node = new DocumentmentNode();
        node.Name = i.ToString();
        for (int j = 0; j < 5; j++)
        {
          FlowDocument doc = new FlowDocument();
          Paragraph p = new Paragraph(new Run(str));
          p.FontSize = 36;
          doc.Blocks.Add(p);
          p.FontSize = 14;
          p.FontStyle = FontStyles.Italic;
          p.TextAlignment = TextAlignment.Left;
          p.Foreground = Brushes.Gray;
          doc.Blocks.Add(p);
          node.DocItems.Add(doc);
        }
        DocItemsTree.Add(node);
      }
     
      InitializeComponent();
      this.DataContext = this;

      ScrollViewer scrollViewer = FindVisualChild<ScrollViewer>(TreeView1);
    }

    private void FlowDocumentScrollViewer_PreviewTouchDown(object sender, TouchEventArgs e)
    {

    }

    private void FlowDocumentScrollViewer_PreviewTouchMove(object sender, TouchEventArgs e)
    {

    }

    private void FlowDocumentScrollViewer_PreviewTouchUp(object sender, TouchEventArgs e)
    {

    }
  }
}
