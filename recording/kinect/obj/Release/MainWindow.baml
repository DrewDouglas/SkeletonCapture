   M S B A M L   `   `   `  ����   BodyBasics-WPF4    .Microsoft.Samples.Kinect.BodyBasics.MainWindow   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     -     � Title$  Skeleton Capture��$	��600��$	��804�� � 	Resources    MediumGreyBrush �� Color  KinectPurpleBrush  KinectBlueBrush��%]   &	        &	      &	 *     �� $ 	#ff6e6e6e���� $ 	#ff52318f���� $ 	#ff00BCF2��.��� $��	10 0 10 0q� � RowDefinitions �� $
��Auto ��� $��* ��� $
��Auto �.+��� $��0��$��Images\Logo.png��$
��Left=�$
��Fill=�$��32��$��81��$��0 10 0 5q��� $��0��$��0,0,-5.4,0.2q�$��Right=�$��Bottom=�#��[  $��Segoe UI)�$��18(�Body Basics�� $��0��$��Images\Status.png��$
��None=�$��Center=�$��0 0 0 5q�(� $��1��$��Center=�.���� $��UniformToFill=����� *ImageSource+�� -   $��	statusBar��$��2��$��Stretch=�$��Bottom=�
�������#��[  .@��� ���� *
StatusText+�� -    ��Name$ startButton��$��Start��$
��Left=�$��665,73.8,0,0q�$��1��$	��Top=�$��75�����	#FF259426�� -   $ 	endButton��$	��End��$
��Left=�$��665,133.8,0,0q�$��1��$	��Top=�$��75�����	#FFD02929�� -    ��Name$ filenameTextBox��$
��Left=�$��23��$��648,192.8,0,0q�$��1�� �� TextWrapping$
 Wrap=�$��	file name��$	��Top=�$	��120��$��2q� � Cursor$ IBeam��