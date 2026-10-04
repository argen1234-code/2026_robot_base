#!/usr/bin/perl
# Strip pixel data from LVGL SquareLine image C files.
# Replaces data array with a minimal stub so images are NOT embedded in Flash.
# Runtime must load pixel data from TF card via tf_image_load().
#
# Usage: perl strip_image_data.pl <image.c>

use strict;
use warnings;

my $file = shift or die "Usage: $0 <image.c>\n";

open(my $fh, '<:raw', $file) or die "$file: $!\n";
my $content;
{ local $/; $content = <$fh>; }
close($fh);

# Replace the data array content: { 0xXX,0xXX,... }  -> { 0 }
$content =~ s/(const\s+LV_ATTRIBUTE_MEM_ALIGN\s+uint8_t\s+\w+_data\[\]\s*=\s*)\{.*?\};
/${1}{ 0 };\n/s or warn "WARNING: could not find data array in $file\n";

# Replace data_size with 0
$content =~ s/(\.data_size\s*=\s*)sizeof\(\w+_data\)/${1}0/g;

# Replace .data pointer with NULL
$content =~ s/(\.data\s*=\s*)\w+_data/${1}NULL/g;

open(my $ofh, '>:raw', $file) or die "$file: $!\n";
print $ofh $content;
close($ofh);
print "Stripped: $file\n";
