#include "brisk/brisk.h"
#include <boost/test/unit_test.hpp>
#include <cstdlib>
#include <bitset>

BOOST_AUTO_TEST_SUITE(BriskTestSuite)

BOOST_AUTO_TEST_CASE(Test_HammingSse)
{
	unsigned char array1[16], array2[16];
	unsigned char *signature1 = &array1[0],
		*signature2 = &array2[0];
	cv::HammingSse hamming_functor;
	int hamming_distance;

	// Test zero signatures
	memset(signature1, 0, 16);
	memset(signature2, 0, 16);
	hamming_distance = hamming_functor(&signature1[0], &signature2[0], 16);

	BOOST_CHECK_EQUAL(hamming_distance, 0);
	
	// Test one argument non zero
	for(int i=0; i<16; i++) {
		memset(signature1, 0, 16);
		signature1[i] = 255;
		hamming_distance = hamming_functor(signature1, signature2, 16);
		BOOST_CHECK_EQUAL(hamming_distance, 8);
		hamming_distance = hamming_functor(signature2, signature1, 16);
		BOOST_CHECK_EQUAL(hamming_distance, 8);
	}
	
	// Test both arguments non zero
	srand( time(NULL) ); //initialize random seed
	for(int run=0; run<1024; run++) {
		memset(signature1, 0, 16);
		memset(signature2, 0, 16);
		int counter = 0;
		for(int i=0; i<16; i++) {
			signature1[i] = static_cast<unsigned char>( rand() );
			signature2[i] = static_cast<unsigned char>( rand() );
			hamming_distance = hamming_functor(signature1, signature2, 16);

			std::bitset<8> xor_bitset(signature1[i] ^ signature2[i]);
			counter += xor_bitset.count();
			
			BOOST_CHECK_EQUAL( hamming_distance, counter );
		}
	}
}

BOOST_AUTO_TEST_SUITE_END()
