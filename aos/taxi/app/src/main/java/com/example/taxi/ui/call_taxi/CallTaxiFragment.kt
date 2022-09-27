package com.example.taxi.ui.call_taxi

import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.appcompat.widget.SearchView
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.api.KakaoAPI
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.DestinationSearch
import com.example.taxi.data.dto.user.destination.DestinationSearchDto
import com.example.taxi.data.dto.user.route.Location
import com.example.taxi.data.dto.user.route.RouteSetting
import com.example.taxi.databinding.FragmentCallTaxiBinding
import com.example.taxi.ui.call_taxi.setting.DestinationSearchListAdapter
import com.example.taxi.utils.constant.KakaoApi
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.view.toast
import com.naver.maps.geometry.LatLng
import com.naver.maps.map.*
import com.naver.maps.map.overlay.Marker
import com.naver.maps.map.overlay.Overlay
import com.naver.maps.map.overlay.OverlayImage
import com.naver.maps.map.overlay.PathOverlay
import com.ssafy.daero.utils.view.getPxFromDp
import dagger.hilt.android.AndroidEntryPoint
import org.json.JSONObject
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import java.text.NumberFormat
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.roundToInt
import kotlin.math.sqrt

@AndroidEntryPoint
class CallTaxiFragment : BaseFragment<FragmentCallTaxiBinding>(R.layout.fragment_call_taxi),
    OnMapReadyCallback {

    private val callTaxiViewModel : CallTaxiViewModel by viewModels()
    private lateinit var startingPoint : Destination
    private lateinit var destination : Destination
    private lateinit var destinationSearchListAdapter: DestinationSearchListAdapter
    var checkState = true
    private var naverMap: NaverMap? = null
    private var uiSettings: UiSettings? = null
    private var markers = mutableListOf<Marker>()
    private var paths = mutableListOf<PathOverlay>()

    private val destinationSearchClickListener: (View, String, String, String, String) -> Unit = { _, place, address, x, y ->
        binding.searchCallTaxi.visibility = View.GONE
        binding.recyclerviewCallTaxiSearch.visibility = View.GONE
        binding.textCallTaxiStart.visibility = View.VISIBLE
        binding.imageCallTaxiForward.visibility = View.VISIBLE
        binding.textCallTaxiDestination.visibility = View.VISIBLE
        binding.searchCallTaxi.setQuery("", false)
        destination = Destination(address,y,place,x)
        binding.textCallTaxiDestination.text = destination.addressName
        checkEnd()
    }

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        initNaverMap()
        return super.onCreateView(inflater, container, savedInstanceState)
    }

    override fun init() {
        //TODO : 선불이면 Bootpay 처리
        initData()
        observerData()
        setOnClickListeners()
    }

    private fun initData() {
        if(arguments?.getParcelable<Destination>("Destination")!=null && arguments?.getParcelable<Destination>("StartingPoint")!=null){
            destination = arguments?.getParcelable<Destination>("Destination") as Destination
            Log.d("목적지",destination.toString())
            startingPoint = arguments?.getParcelable<Destination>("StartingPoint") as Destination
            Log.d("출발지",startingPoint.toString())
            binding.textCallTaxiDestination.text = destination.addressName
            binding.textCallTaxiStart.text = startingPoint.addressName
        }
    }

    private fun observerData(){
        callTaxiViewModel.routeSetting.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    //binding.progressBar.hide()
                    toast("경로를 설정 중입니다. 잠시만 기다려 주세요.")
                    callTaxiViewModel.getRoute()
                }
            }
        }
        callTaxiViewModel.route.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    //binding.progressBar.hide()
                    val json = requireActivity().assets.open("node_set.json").reader().readText()
                    val node = JSONObject(json)
                    val location = mutableListOf<Location>()
                    for(i in state.data){
                        val pathNode = node.getJSONArray(i)
                        location.add(Location(pathNode.getDouble(1).toString(), pathNode.getDouble(0).toString()))
                    }
                    callTaxiViewModel.getDistance()
                    deleteMarkers()
                    deletePaths()
                    drawMarkers(location)
                    drawPolyline(location)
                }
            }
        }
        callTaxiViewModel.distance.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    //binding.progressBar.hide()
                    var str = ((state.data.toDouble()/1000.0) * 100.0).roundToInt() / 100.0
                    binding.textCallTaxiDistance.text = str.toString() +"Km"
                    getFee(str)
                }
            }
        }
    }

    private fun getFee(distance: Double){
        var distance = distance
        val numberFormat: NumberFormat = NumberFormat.getInstance()
        if(distance <= 3){
            val str = numberFormat.format(3000)
            binding.textCallTaxiCash.text = str + " 원"
        }else{
            distance -= 3
            var res = distance/0.16
            var fee = 3000 + (res.toInt()*100)
            val str = numberFormat.format(fee)
            binding.textCallTaxiCash.text = str + " 원"
        }
    }

    private fun deleteMarkers() {
        if (markers.isNotEmpty()) {
            markers.forEach {
                it.map = null
            }
        }
        markers = mutableListOf()
    }

    private fun deletePaths() {
        if (paths.isNotEmpty()) {
            paths.forEach {
                it.map = null
            }
        }
        paths = mutableListOf()
    }

    private fun drawMarkers(location: List<Location>) {
        if (location.isNotEmpty()) {
            markers.add(createMarker(location[0], 0))
            markers.add(createMarker(location[location.lastIndex], location.lastIndex))

            naverMap?.moveCamera(
                CameraUpdate.scrollTo(
                    LatLng(
                        location[0].lati.toDouble(),
                        location[0].long.toDouble()
                    )
                )
            )
            naverMap?.moveCamera(CameraUpdate.zoomTo(15.0))
        }
    }

    private fun createMarker(location: Location, index: Int): Marker {
        return Marker().apply {
            position = LatLng(location.lati.toDouble(), location.long.toDouble())    // 마커 좌표
            icon = OverlayImage.fromResource(R.drawable.ic_marker)
            if(index==0){
                iconTintColor = requireActivity().getColor(R.color.greenTextColor)// 마커 색깔
            }else{
                iconTintColor = requireActivity().getColor(R.color.primaryColor)// 마커 색깔
            }
            width = requireContext().getPxFromDp(40f)   // 마커 가로 크기
            height = requireContext().getPxFromDp(40f)  // 마커 세로 크기
            zIndex = 0  // 마커 높이
            onClickListener = Overlay.OnClickListener {     // 마커 클릭 리스너
                // todo: location_seq 이용해서 트립스탬프 상세화면으로 이동
                return@OnClickListener true
            }
            isHideCollidedMarkers = true    // 겹치면 다른 마커 숨기기
            map = naverMap  // 지도에 마커 표시
        }
    }

    private fun drawPolyline(location: List<Location>) {
        if (location.isNotEmpty()) {
            if (location.size >= 2) {  // 한 여행에서 두 개 이상 여행지 방문했을 때만 경로 그리기
                paths.add(PathOverlay().apply {
                    color = requireActivity().getColor(R.color.primaryColor) // 경로 색깔
                    outlineColor = requireActivity().getColor(R.color.primaryColor) // 경로 색깔
                    outlineWidth = requireContext().getPxFromDp(1.5f) // 경로 두께
                    var list = mutableListOf<LatLng>()
                    for(i in location){
                        list.add(LatLng(i.lati.toDouble(), i.long.toDouble()))
                    }
                    coords = list  // 경로 좌표
                    map = naverMap
                })
            }
        }
    }

    private fun setOnClickListeners() {
        binding.textCallTaxiStart.setOnClickListener {
            findNavController().navigate(R.id.action_callTaxiFragment_to_startPointSettingFragment
                ,bundleOf("Destination" to destination))
        }
        binding.textCallTaxiDestination.setOnClickListener {
            binding.searchCallTaxi.visibility = View.VISIBLE
            binding.recyclerviewCallTaxiSearch.visibility = View.VISIBLE
            binding.textCallTaxiStart.visibility = View.GONE
            binding.imageCallTaxiForward.visibility = View.GONE
            binding.textCallTaxiDestination.visibility = View.GONE
            checkState = true
        }
        binding.imageCallTaxiFirstPayment.setOnClickListener {

        }
        binding.imageCallTaxiLatePayment.setOnClickListener {
            findNavController().navigate(R.id.action_callTaxiFragment_to_waitingCallTaxiFragment,
                bundleOf("Destination" to destination, "StartingPoint" to startingPoint)
            )
        }
        binding.searchCallTaxi.setOnQueryTextListener(object : SearchView.OnQueryTextListener {
            override fun onQueryTextSubmit(query: String?): Boolean {
                // 검색 버튼 누를 때 호출

                return true
            }

            override fun onQueryTextChange(newText: String?): Boolean {
                if (newText != null && newText != "") {
                    searchKeyword(newText)
                }
                return true
            }
        })
        binding.imageCallTaxiBack.setOnClickListener {
            requireActivity().onBackPressed()
        }
    }

    private fun searchKeyword(keyword: String) {
        val retrofit = Retrofit.Builder()   // Retrofit 구성
            .baseUrl(KakaoApi.BASE_URL)
            .addConverterFactory(GsonConverterFactory.create())
            .build()
        val api = retrofit.create(KakaoAPI::class.java)   // 통신 인터페이스를 객체로 생성
        val call = api.getSearchKeyword(KakaoApi.API_KEY, keyword)   // 검색 조건 입력

        // API 서버에 요청
        call.enqueue(object: Callback<DestinationSearchDto> {
            override fun onResponse(
                call: Call<DestinationSearchDto>,
                response: Response<DestinationSearchDto>
            ) {
                // 통신 성공 (검색 결과는 response.body()에 담겨있음)
                Log.d("Test", "Raw: ${response.raw()}")
                Log.d("Test", "Body: ${response.body()}")
                if(response.body()!!.documents != null){
                    if(checkState){
                        getDistance(response.body()!!.documents)
                    }else{
                        initSearchAdapter(response.body()!!.documents)
                    }
                }
            }

            override fun onFailure(call: Call<DestinationSearchDto>, t: Throwable) {
                // 통신 실패
                Log.w("MainActivity", "통신 실패: ${t.message}")
            }
        })
    }

    private fun getDistance(list : List<DestinationSearch>) {
        if(startingPoint!=null){
            for(i in list){
                var newX = ( kotlin.math.cos(startingPoint.latitude.toDouble()) * 6400 * 2 * 3.14 / 360 ) * abs(startingPoint.longitude.toDouble() - i.x.toDouble())
                var newY = 111 * abs(startingPoint.longitude.toDouble() - i.y.toDouble())

                i.distance = ((sqrt(newX.pow(2)+newY.pow(2)) * 100.0).roundToInt() / 100.0).toString()+"Km"
            }
        }
        initSearchAdapter(list)
    }

    private fun initSearchAdapter(list : List<DestinationSearch>){
        destinationSearchListAdapter = DestinationSearchListAdapter().apply {
            onItemClickListener = destinationSearchClickListener
        }
        binding.recyclerviewCallTaxiSearch.apply {
            adapter = destinationSearchListAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
        }
        destinationSearchListAdapter.updateList(list)
    }
    
    private fun checkEnd(){
        if(binding.textCallTaxiStart.text !="" && binding.textCallTaxiDestination.text != ""){
            callTaxiViewModel.addRouteSetting(RouteSetting(destination = Location(
                lati = destination.latitude, long = destination.longitude),
                startingPoint = Location(lati = startingPoint.latitude, long = startingPoint.longitude)))
            callTaxiViewModel.getRoute()
        }
    }

    private fun initNaverMap() {
        val _naverMap =
            childFragmentManager.findFragmentById(R.id.fragmentContainer_call_taxi) as MapFragment?
                ?: MapFragment.newInstance().also {
                    childFragmentManager.beginTransaction()
                        .add(R.id.fragmentContainer_call_taxi, it)
                        .commit()
                }
        _naverMap.getMapAsync(this)
    }

    override fun onMapReady(_naverMap: NaverMap) {
        naverMap = _naverMap

        setNaverMapUI()
        getRoute()
    }

    private fun setNaverMapUI() {
        naverMap?.apply {
            isLiteModeEnabled = false // 가벼운 지도 모드 (건물 내부 상세 표시 X)

            this@CallTaxiFragment.uiSettings = this.uiSettings.apply {
                isCompassEnabled = false // 나침반 비활성화
                isZoomControlEnabled = false // 확대 축소 버튼 비활성화
                isScaleBarEnabled = false // 스케일 바 비활성화
                isLocationButtonEnabled = false // 기본 내 위치 버튼 비활성화
            }
        }
    }

    private fun getRoute() {
        callTaxiViewModel.getDistance()
        if(arguments?.getParcelable<Destination>("Destination")!=null && arguments?.getParcelable<Destination>("StartingPoint")!=null){
            destination = arguments?.getParcelable<Destination>("Destination") as Destination
            startingPoint = arguments?.getParcelable<Destination>("StartingPoint") as Destination
            callTaxiViewModel.addRouteSetting(RouteSetting(destination = Location(
                lati = destination.latitude, long = destination.longitude),
            startingPoint = Location(lati = startingPoint.latitude, long = startingPoint.longitude)))
        }
    }

}