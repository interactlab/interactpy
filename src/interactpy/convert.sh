# Converts a folder of .ogv files to .mp4 files. FFMPEG is required.

# Usage: ./convert.sh /path/to/folder/of/ogv/videos/

# The script will place the .mp4 videos in a folder called "mp4_output" inside
# the folder containing the ogv files being converted

cd $1
for file in ./*
do
  ffmpeg -i $file -c:v libx264 -preset veryslow -crf 22 -c:a libmp3lame -qscale:a 2 -ac 2 -ar 44100 "${file:: -4}.mp4"
done
mkdir -p mp4_output
mv *.mp4 mp4_output 
