%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RV - Vaja 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Pocisti "Workspace" in "Command Window"
clear all;
close all;
clc;



%% 1.0 - Branje datotek in izpis vrednosti RGB barvnih kanalov 

% Branje datotek
Slika1 = imread("lena.tiff");
Slika2 = imread("baboon.tiff");
Video = VideoReader("Holywood.avi");


% Izpis vrednosti barvnih kanalov - "Lena"
Slika1_R = Slika1(100, 350, 1);
Slika1_G = Slika1(100, 350, 2);
Slika1_B = Slika1(100, 350, 3);

disp('RGB kanali "Lena": ');
disp([Slika1_R, Slika1_G, Slika1_B]);


% Izpis vrednosti barvnih kanalov - "Baboon"
Slika2_R = Slika2(100, 350, 1);
Slika2_G = Slika2(100, 350, 2);
Slika2_B = Slika2(100, 350, 3);

disp('RGB kanali "Baboon": ');
disp([Slika2_R, Slika2_G, Slika2_B]);


% Izpis vrednosti barvnih kanalov - "Holywood"
Video_frames = read(Video, [50 60]);
Video_RGB = zeros(10, 3);

for i = 1:10
    for j = 1:3
        Video_RGB(i, j) = Video_frames(100, 350, j, i);
    end
end

disp('RGB kanali "Holywood": ');
disp(Video_RGB);


% Graficni prikaz rezultatov
figure;

subplot(2, 2, 1);
imshow(uint8(Slika1));

subplot(2, 2, 2);
imshow(uint8(Slika2));

subplot(2, 2, 3:4);
while hasFrame(Video)
    frame = readFrame(Video);
    imshow(uint8(frame));
    pause(1 / (10 * Video.FrameRate));
end

close all;



%% 1.1. - Pretvorba v sivinski prostor

% Pretvorba iz RGB v sivinski prostor - "Lena"
Slika1_Siva = 0.299 * Slika1(:,:,1) + 0.587 * Slika1(:,:,2) + 0.114 * Slika1(:,:,3);

Slika1_Siva = repmat(Slika1_Siva, 1);
imwrite(Slika1_Siva, "Lena_Siva.tiff");


% Pretvorba iz RGB v sivinski prostor - "Baboon"
Slika2_Siva = 0.299 * Slika2(:,:,1) + 0.587 * Slika2(:,:,2) + 0.114 * Slika2(:,:,3);

Slika2_Siva = repmat(Slika2_Siva, 1);
imwrite(Slika2_Siva, "Baboon_Siva.tiff");


% Pretvorba iz RGB v sivinski prostor - "Holywood"
Video_frames = read(Video);
Video_Siv_frame = 0.299 * Video_frames(:,:,1,:) + 0.587 * Video_frames(:,:,2,:) + 0.114 * Video_frames(:,:,3,:);

Video_Siv = VideoWriter("Holywood_Siv.avi", "Grayscale AVI");
open(Video_Siv);
writeVideo(Video_Siv, Video_Siv_frame);
close(Video_Siv);


% Graficni prikaz rezultatov
Video_Siv = VideoReader("Holywood_Siv.avi");
figure;

subplot(2, 2, 1);
imshow(uint8(Slika1_Siva));

subplot(2, 2, 2);
imshow(uint8(Slika2_Siva));

subplot(2, 2, 3:4);
while hasFrame(Video_Siv)
    frame = readFrame(Video_Siv);
    imshow(uint8(frame));
    pause(1 / (10 * Video_Siv.FrameRate));
end

close all;



%% 1.2 - Pretvorba nazaj v RGB prostor

% Funkcija za pretvorbo vrednostni barvnih kanalov pikslov
function RGB = pretvorba(Sivo)
    zelena = [153, 255, 102];  
    oranzna = [255, 153, 51];  
    rdeca = [255, 80, 80];     

    Slika_Visina = size(Sivo, 1);  
    Slika_Sirina = size(Sivo, 2);  

    RGB = zeros(Slika_Visina, Slika_Sirina, 3, 'uint8');

    for i = 1:Slika_Visina
        for j = 1:Slika_Sirina
            if Sivo(i, j) <= 100
                RGB(i, j, 1) = zelena(1);
                RGB(i, j, 2) = zelena(2);
                RGB(i, j, 3) = zelena(3);

            elseif (Sivo(i, j) > 100) && (Sivo(i, j) <= 200)
                RGB(i, j, 1) = oranzna(1);
                RGB(i, j, 2) = oranzna(2);
                RGB(i, j, 3) = oranzna(3);

            elseif Sivo(i, j) > 200
                RGB(i, j, 1) = rdeca(1);
                RGB(i, j, 2) = rdeca(2);
                RGB(i, j, 3) = rdeca(3);
            end
        end
    end
end

% Pretvorba iz sivinskega v RGB prostor - "Lena"
Slika1_RGB = pretvorba(Slika1_Siva);
imwrite(Slika1_RGB, "Lena_RGB.tiff");


%Pretvorba iz sivinskega v RGB prostor - "Baboon"
Slika2_RGB = pretvorba(Slika2_Siva);
imwrite(Slika2_RGB, "Baboon_RGB.tiff");


% Pretvorba iz sivinskega v RGB prostor - "Holywood"
Video_RGB = VideoWriter("Holywood_RGB.avi", "Uncompressed AVI");
open(Video_RGB);

% Zakaj se Video_Siv ne prebere v funkciji pravilno, ce se video ne
% reinicializira!??!?!?!
Video_Siv = VideoReader("Holywood_Siv.avi");

while hasFrame(Video_Siv)
    frame_siv = readFrame(Video_Siv);
    frame_RGB = pretvorba(frame_siv);
    writeVideo(Video_RGB, frame_RGB);
end

close(Video_RGB);


% Graficni prikaz rezultatov
Video_RGB = VideoReader("Holywood_RGB.avi");
figure;

subplot(2, 2, 1);
imshow(uint8(Slika1_RGB));

subplot(2, 2, 2);
imshow(uint8(Slika2_RGB));

subplot(2, 2, 3:4);
while hasFrame(Video_RGB)
    frame = readFrame(Video_RGB);
    imshow(uint8(frame));
    pause(1 / (10 * Video_RGB.FrameRate));
end

close all;


