/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <random>
#include <iostream>
#include <unittest++/UnitTest++.h>

#include "StreetsDatabaseAPI.h"
#include "m1.h"
#include "m3.h"
#include "m4.h"

#include "unit_test_util.h"
#include "courier_verify.h"

using ece297test::relative_error;
using ece297test::courier_path_is_legal;


SUITE(simple_legality_toronto_canada_public) {
    TEST(simple_legality_toronto_canada) {
        std::vector<DeliveryInfo> deliveries;
        std::vector<unsigned> depots;
        std::vector<unsigned> result_path;
        float turn_penalty;
        
deliveries = {DeliveryInfo(64489, 10192), DeliveryInfo(66348, 47055)};
        depots = {75020, 59249};
        turn_penalty = 15;
        result_path = traveling_courier(deliveries, depots, turn_penalty);
        CHECK(courier_path_is_legal(deliveries, depots, result_path));
        
    } //simple_legality_toronto_canada

} //simple_legality_toronto_canada_public