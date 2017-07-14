#include "PIDController.h"
#include "RotationalController.h"
#include <cmath>
#include <iostream>
#include <string>

float const FLOAT_COMP_THRESHOLD = 0.001;
bool runTests();
bool assertFloatEquals(float actual, float expected);

int tests_passed = 0;
int tests_total = 0;


bool assertFloatEquals(float actual, float expected) {
    if (fabs(actual-expected) < FLOAT_COMP_THRESHOLD) {
        return true;
    }
    else {
        std::cout << "\nASSERT FLOAT EQUALS FAILED\n";
        std::cout << "EXPECTED: ";
        std::cout << expected;
        std::cout << "\nACTUAL: ";
        std::cout << actual;
        std::cout << "\n";
    }
}

int main(int argc, char **argv) {
    runTests();
    std::cout << tests_passed;
    std::cout << "/";
    std::cout << tests_total;
    std::cout << " Tests passed\n";
    return 0;
}
bool testRotationalController(std::string label, pose current_pos, pose goal_pos, float expected)
{
    RotationalController rotational_controller;
    tests_total++;
    std::cout << "\nBeginning Test: ";
    std::cout << label;

    float angular_velocity = rotational_controller.calculateVelocity(current_pos,goal_pos);
    if (!assertFloatEquals(angular_velocity,expected))
    {
        std::cout << "\n";
        std::cout << label;
        std::cout << " FAILED\n";
        return false;
    }
    std::cout << "\n";
    std::cout << label;
    std::cout << " PASSED\n";
    tests_passed++;
    return true;
}

bool runTests()
{
//    testRotationalController("Test1",{0.0,0.0,0},{0,1,1.5708},0.3);
//    testRotationalController("Test2",{0.0,0.0,0.03},{0,1,1.5708},0.3);
//    testRotationalController("Test3",{0.0,0.0,0.06},{0,1,1.5708},0.3);
//    testRotationalController("Test4",{0.0,0.0,0.09},{0,1,1.5708},0.3);
//    testRotationalController("Test5",{0.0,0.0,0.12},{0,1,1.5708},0.3);
//    testRotationalController("Test6",{0.0,0.0,0.15},{0,1,1.5708},0.3);
//    testRotationalController("Test7",{0.0,0.0,0.18},{0,1,1.5708},0.3);
//    testRotationalController("Test8",{0.0,0.0,0.21},{0,1,1.5708},0.3);
//    testRotationalController("Test9",{0.0,0.0,0.24},{0,1,1.5708},0.3);
//    testRotationalController("Test10",{0.0,0.0,0.27},{0,1,1.5708},0.3);
//    testRotationalController("Test11",{0.0,0.0,0.3},{0,1,1.5708},0.3);
//    testRotationalController("Test12",{0.0,0.0,0.33},{0,1,1.5708},0.3);
//    testRotationalController("Test13",{0.0,0.0,0.36},{0,1,1.5708},0.3);
//    testRotationalController("Test14",{0.0,0.0,0.39},{0,1,1.5708},0.3);
//    testRotationalController("Test15",{0.0,0.0,0.42},{0,1,1.5708},0.3);
//    testRotationalController("Test16",{0.0,0.0,0.45},{0,1,1.5708},0.3);
//    testRotationalController("Test17",{0.0,0.0,0.48},{0,1,1.5708},0.3);
//    testRotationalController("Test18",{0.0,0.0,0.51},{0,1,1.5708},0.3);
//    testRotationalController("Test19",{0.0,0.0,0.54},{0,1,1.5708},0.3);
//    testRotationalController("Test20",{0.0,0.0,0.57},{0,1,1.5708},0.3);
//    testRotationalController("Test21",{0.0,0.0,0.6},{0,1,1.5708},0.3);
//    testRotationalController("Test22",{0.0,0.0,0.63},{0,1,1.5708},0.3);
//    testRotationalController("Test23",{0.0,0.0,0.66},{0,1,1.5708},0.3);
//    testRotationalController("Test24",{0.0,0.0,0.69},{0,1,1.5708},0.3);
//    testRotationalController("Test25",{0.0,0.0,0.72},{0,1,1.5708},0.3);
//    testRotationalController("Test26",{0.0,0.0,0.75},{0,1,1.5708},0.3);
//    testRotationalController("Test27",{0.0,0.0,0.78},{0,1,1.5708},0.3);
//    testRotationalController("Test28",{0.0,0.0,0.81},{0,1,1.5708},0.3);
//    testRotationalController("Test29",{0.0,0.0,0.84},{0,1,1.5708},0.3);
//    testRotationalController("Test30",{0.0,0.0,0.87},{0,1,1.5708},0.3);
//    testRotationalController("Test31",{0.0,0.0,0.9},{0,1,1.5708},0.3);
//    testRotationalController("Test32",{0.0,0.0,0.93},{0,1,1.5708},0.3);
//    testRotationalController("Test33",{0.0,0.0,0.96},{0,1,1.5708},0.3);
//    testRotationalController("Test34",{0.0,0.0,0.99},{0,1,1.5708},0.3);
//    testRotationalController("Test35",{0.0,0.0,1.02},{0,1,1.5708},0.3);
//    testRotationalController("Test36",{0.0,0.0,1.05},{0,1,1.5708},0.3);
//    testRotationalController("Test37",{0.0,0.0,1.08},{0,1,1.5708},0.3);
//    testRotationalController("Test38",{0.0,0.0,1.11},{0,1,1.5708},0.3);
//    testRotationalController("Test39",{0.0,0.0,1.14},{0,1,1.5708},0.3);
    testRotationalController("Test40",{0.0,0.0,1.17},{0,1,1.5708},0.3);
    testRotationalController("Test41",{0.0,0.0,1.2},{0,1,1.5708},0.2781);
//    testRotationalController("Test42",{0.0,0.0,1.2278},{0,1,1.5708},0.25724);
//    testRotationalController("Test43",{0.0,0.0,1.2535},{0,1,1.5708},0.23795);
//    testRotationalController("Test44",{0.0,0.0,1.2773},{0,1,1.5708},0.2201);
//    testRotationalController("Test45",{0.0,0.0,1.2993},{0,1,1.5708},0.20359);
//    testRotationalController("Test46",{0.0,0.0,1.3197},{0,1,1.5708},0.18832);
//    testRotationalController("Test47",{0.0,0.0,1.3385},{0,1,1.5708},0.1742);
//    testRotationalController("Test48",{0.0,0.0,1.356},{0,1,1.5708},0.16113);
//    testRotationalController("Test49",{0.0,0.0,1.3721},{0,1,1.5708},0.14905);
//    testRotationalController("Test50",{0.0,0.0,1.387},{0,1,1.5708},0.13787);
//    testRotationalController("Test51",{0.0,0.0,1.4008},{0,1,1.5708},0.12753);
//    testRotationalController("Test52",{0.0,0.0,1.4135},{0,1,1.5708},0.11797);
//    testRotationalController("Test53",{0.0,0.0,1.4253},{0,1,1.5708},0.10912);
//    testRotationalController("Test54",{0.0,0.0,1.4362},{0,1,1.5708},0.10093);
//    testRotationalController("Test55",{0.0,0.0,1.4463},{0,1,1.5708},0.093364);
//    testRotationalController("Test56",{0.0,0.0,1.4556},{0,1,1.5708},0.086362);
//    testRotationalController("Test57",{0.0,0.0,1.4643},{0,1,1.5708},0.079885);
//    testRotationalController("Test58",{0.0,0.0,1.4723},{0,1,1.5708},0.073893);
//    testRotationalController("Test59",{0.0,0.0,1.4797},{0,1,1.5708},0.068351);
//    testRotationalController("Test60",{0.0,0.0,1.4865},{0,1,1.5708},0.063225);
//    testRotationalController("Test61",{0.0,0.0,1.4928},{0,1,1.5708},0.058483);
//    testRotationalController("Test62",{0.0,0.0,1.4987},{0,1,1.5708},0.054097);
//    testRotationalController("Test63",{0.0,0.0,1.5041},{0,1,1.5708},0.05004);
//    testRotationalController("Test64",{0.0,0.0,1.5091},{0,1,1.5708},0.046287);
//    testRotationalController("Test65",{0.0,0.0,1.5137},{0,1,1.5708},0.042815);
//    testRotationalController("Test66",{0.0,0.0,1.518},{0,1,1.5708},0.039604);
//    testRotationalController("Test67",{0.0,0.0,1.522},{0,1,1.5708},0.036634);
//    testRotationalController("Test68",{0.0,0.0,1.5256},{0,1,1.5708},0.033886);
//    testRotationalController("Test69",{0.0,0.0,1.529},{0,1,1.5708},0.031345);
//    testRotationalController("Test70",{0.0,0.0,1.5321},{0,1,1.5708},0.028994);
//    testRotationalController("Test71",{0.0,0.0,1.535},{0,1,1.5708},0.026819);
//    testRotationalController("Test72",{0.0,0.0,1.5377},{0,1,1.5708},0.024808);
//    testRotationalController("Test73",{0.0,0.0,1.5402},{0,1,1.5708},0.022947);
//    testRotationalController("Test74",{0.0,0.0,1.5425},{0,1,1.5708},0.021226);
//    testRotationalController("Test75",{0.0,0.0,1.5446},{0,1,1.5708},0.019634);
//    testRotationalController("Test76",{0.0,0.0,1.5466},{0,1,1.5708},0.018162);
//    testRotationalController("Test77",{0.0,0.0,1.5484},{0,1,1.5708},0.0168);
//    testRotationalController("Test78",{0.0,0.0,1.5501},{0,1,1.5708},0.01554);
//    testRotationalController("Test79",{0.0,0.0,1.5516},{0,1,1.5708},0.014374);
//    testRotationalController("Test80",{0.0,0.0,1.5531},{0,1,1.5708},0.013296);
//    testRotationalController("Test81",{0.0,0.0,1.5544},{0,1,1.5708},0.012299);
//    testRotationalController("Test82",{0.0,0.0,1.5556},{0,1,1.5708},0.011376);
//    testRotationalController("Test83",{0.0,0.0,1.5568},{0,1,1.5708},0.010523);
//    testRotationalController("Test84",{0.0,0.0,1.5578},{0,1,1.5708},0.009734);
//    testRotationalController("Test85",{0.0,0.0,1.5588},{0,1,1.5708},0.0090039);
//    testRotationalController("Test86",{0.0,0.0,1.5597},{0,1,1.5708},0.0083287);
//    testRotationalController("Test87",{0.0,0.0,1.5605},{0,1,1.5708},0.007704);
//    testRotationalController("Test88",{0.0,0.0,1.5613},{0,1,1.5708},0.0071262);
//    testRotationalController("Test89",{0.0,0.0,1.562},{0,1,1.5708},0.0065917);
//    testRotationalController("Test90",{0.0,0.0,1.5627},{0,1,1.5708},0.0060974);
//    testRotationalController("Test91",{0.0,0.0,1.5633},{0,1,1.5708},0.0056401);
//    testRotationalController("Test92",{0.0,0.0,1.5638},{0,1,1.5708},0.0052171);
//    testRotationalController("Test93",{0.0,0.0,1.5644},{0,1,1.5708},0.0048258);
//    testRotationalController("Test94",{0.0,0.0,1.5648},{0,1,1.5708},0.0044638);
//    testRotationalController("Test95",{0.0,0.0,1.5653},{0,1,1.5708},0.0041291);
//    testRotationalController("Test96",{0.0,0.0,1.5657},{0,1,1.5708},0.0038194);
//    testRotationalController("Test97",{0.0,0.0,1.5661},{0,1,1.5708},0.0035329);
//    testRotationalController("Test98",{0.0,0.0,1.5664},{0,1,1.5708},0.003268);
//    testRotationalController("Test99",{0.0,0.0,1.5668},{0,1,1.5708},0.0030229);
//    testRotationalController("Test100",{0.0,0.0,1.5671},{0,1,1.5708},0.0027961);
//    testRotationalController("Test101",{0.0,0.0,1.5673},{0,1,1.5708},0.0025864);
//    testRotationalController("Test102",{0.0,0.0,1.5676},{0,1,1.5708},0.0023924);
//    testRotationalController("Test103",{0.0,0.0,1.5678},{0,1,1.5708},0.002213);
//    testRotationalController("Test104",{0.0,0.0,1.5681},{0,1,1.5708},0.002047);
//    testRotationalController("Test105",{0.0,0.0,1.5683},{0,1,1.5708},0.0018935);
//    testRotationalController("Test106",{0.0,0.0,1.5685},{0,1,1.5708},0.0017515);
//    testRotationalController("Test107",{0.0,0.0,1.5686},{0,1,1.5708},0.0016201);
//    testRotationalController("Test108",{0.0,0.0,1.5688},{0,1,1.5708},0.0014986);
//    testRotationalController("Test109",{0.0,0.0,1.5689},{0,1,1.5708},0.0013862);
//    testRotationalController("Test110",{0.0,0.0,1.5691},{0,1,1.5708},0.0012823);
//    testRotationalController("Test111",{0.0,0.0,1.5692},{0,1,1.5708},0.0011861);
//    testRotationalController("Test112",{0.0,0.0,1.5693},{0,1,1.5708},0.0010971);
//    testRotationalController("Test113",{0.0,0.0,1.5694},{0,1,1.5708},0.0010148);
//    testRotationalController("Test114",{0.0,0.0,1.5695},{0,1,1.5708},0.00093874);
//    testRotationalController("Test115",{0.0,0.0,1.5696},{0,1,1.5708},0.00086833);
//    testRotationalController("Test116",{0.0,0.0,1.5697},{0,1,1.5708},0.00080321);
//    testRotationalController("Test117",{0.0,0.0,1.5698},{0,1,1.5708},0.00074297);
//    testRotationalController("Test118",{0.0,0.0,1.5699},{0,1,1.5708},0.00068724);
//    testRotationalController("Test119",{0.0,0.0,1.5699},{0,1,1.5708},0.0006357);
//    testRotationalController("Test120",{0.0,0.0,1.57},{0,1,1.5708},0.00058802);
//    testRotationalController("Test121",{0.0,0.0,1.5701},{0,1,1.5708},0.00054392);
//    testRotationalController("Test122",{0.0,0.0,1.5701},{0,1,1.5708},0.00050313);
//    testRotationalController("Test123",{0.0,0.0,1.5702},{0,1,1.5708},0.00046539);
//    testRotationalController("Test124",{0.0,0.0,1.5702},{0,1,1.5708},0.00043049);
//    testRotationalController("Test125",{0.0,0.0,1.5703},{0,1,1.5708},0.0003982);
//    testRotationalController("Test126",{0.0,0.0,1.5703},{0,1,1.5708},0.00036834);
//    testRotationalController("Test127",{0.0,0.0,1.5703},{0,1,1.5708},0.00034071);
//    testRotationalController("Test128",{0.0,0.0,1.5704},{0,1,1.5708},0.00031516);
//    testRotationalController("Test129",{0.0,0.0,1.5704},{0,1,1.5708},0.00029152);
//    testRotationalController("Test130",{0.0,0.0,1.5704},{0,1,1.5708},0.00026966);
//    testRotationalController("Test131",{0.0,0.0,1.5705},{0,1,1.5708},0.00024943);
//    testRotationalController("Test132",{0.0,0.0,1.5705},{0,1,1.5708},0.00023072);
//    testRotationalController("Test133",{0.0,0.0,1.5705},{0,1,1.5708},0.00021342);
//    testRotationalController("Test134",{0.0,0.0,1.5705},{0,1,1.5708},0.00019741);
//    testRotationalController("Test135",{0.0,0.0,1.5706},{0,1,1.5708},0.00018261);
//    testRotationalController("Test136",{0.0,0.0,1.5706},{0,1,1.5708},0.00016891);
//    testRotationalController("Test137",{0.0,0.0,1.5706},{0,1,1.5708},0.00015624);
//    testRotationalController("Test138",{0.0,0.0,1.5706},{0,1,1.5708},0.00014453);
//    testRotationalController("Test139",{0.0,0.0,1.5706},{0,1,1.5708},0.00013369);
//    testRotationalController("Test140",{0.0,0.0,1.5706},{0,1,1.5708},0.00012366);
//    testRotationalController("Test141",{0.0,0.0,1.5706},{0,1,1.5708},0.00011439);
//    testRotationalController("Test142",{0.0,0.0,1.5707},{0,1,1.5708},0.00010581);
//    testRotationalController("Test143",{0.0,0.0,1.5707},{0,1,1.5708},9.7871e-05);
//    testRotationalController("Test144",{0.0,0.0,1.5707},{0,1,1.5708},9.0531e-05);
//    testRotationalController("Test145",{0.0,0.0,1.5707},{0,1,1.5708},8.3741e-05);
//    testRotationalController("Test146",{0.0,0.0,1.5707},{0,1,1.5708},7.746e-05);
//    testRotationalController("Test147",{0.0,0.0,1.5707},{0,1,1.5708},7.1651e-05);
//    testRotationalController("Test148",{0.0,0.0,1.5707},{0,1,1.5708},6.6277e-05);
//    testRotationalController("Test149",{0.0,0.0,1.5707},{0,1,1.5708},6.1306e-05);
//    testRotationalController("Test150",{0.0,0.0,1.5707},{0,1,1.5708},5.6708e-05);

}
