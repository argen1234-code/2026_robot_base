#!/usr/bin/perl
use strict;
use warnings;
use Encode;

# Convert Chinese characters in C string literals to UTF-8 \x escape sequences.
# Input: C source file (any encoding)
# Output: Pure-ASCII C source file with UTF-8 hex escapes for non-ASCII chars
# Usage: perl cn_to_utf8_escapes.pl <input.c> [output.c]

my $input_file = shift or die "Usage: $0 <input.c> [output.c]\n";
my $output_file = shift || $input_file;

open(my $fh, '<:raw', $input_file) or die "Cannot open $input_file: $!\n";
my $content;
{
    local $/;
    $content = <$fh>;
}
close($fh);

# First, detect the encoding. Try to read as UTF-8; if it fails, assume GBK.
my $decoded;
eval {
    $decoded = decode('UTF-8', $content, Encode::FB_CROAK);
};
if ($@) {
    # Not valid UTF-8, assume GBK (the previous conversion)
    $decoded = decode('GBK', $content);
    print "Detected encoding: GBK (converting to UTF-8 escapes)\n";
} else {
    print "Detected encoding: UTF-8\n";
}

# Replace non-ASCII characters in string literals with \x escapes
# We look inside double-quoted strings and replace chars with codepoint > 127
$decoded =~ s{("(?:[^"\\]|\\.)*")}{
    my $str = $1;
    # Replace each non-ASCII char with UTF-8 hex escape
    $str =~ s{([\x{80}-\x{10FFFF}])}{
        my $ch = $1;
        my @bytes = unpack('C*', encode('UTF-8', $ch));
        join('', map { sprintf("\\x%02X", $_) } @bytes);
    }ge;
    $str;
}ge;

# Write as pure ASCII
open(my $ofh, '>:raw', $output_file) or die "Cannot write $output_file: $!\n";
print $ofh $decoded;
close($ofh);

print "Done: $output_file (pure ASCII with UTF-8 escapes)\n";
