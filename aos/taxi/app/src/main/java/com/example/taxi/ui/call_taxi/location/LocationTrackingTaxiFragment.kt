package com.example.taxi.ui.call_taxi.location

import com.example.taxi.R
import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.viewModels
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.route.Location
import com.example.taxi.data.dto.user.route.RouteSetting
import com.example.taxi.databinding.FragmentLocationTrackingTaxiBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.call_taxi.CallTaxiViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.view.toast
import com.naver.maps.geometry.LatLng
import com.naver.maps.map.*
import com.naver.maps.map.overlay.*
import com.ssafy.daero.utils.view.getPxFromDp
import dagger.hilt.android.AndroidEntryPoint
import org.json.JSONObject


@AndroidEntryPoint
class LocationTrackingTaxiFragment : BaseFragment<FragmentLocationTrackingTaxiBinding>(R.layout.fragment_location_tracking_taxi),
    OnMapReadyCallback {

    private val callTaxiViewModel : CallTaxiViewModel by viewModels()
    var checkState = true
    private var naverMap: NaverMap? = null
    private var uiSettings: UiSettings? = null
    private var markers = mutableListOf<Marker>()
    private var paths = mutableListOf<PathOverlay>()
    private var infoWindow = InfoWindow()
    private var rootView: ViewGroup? = null

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        rootView = container
        initNaverMap()
        return super.onCreateView(inflater, container, savedInstanceState)
    }

    override fun init() {
        observerData()
        setOnClickListeners()
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
                    deleteMarkers()
                    deletePaths()
                    drawMarkers(location)
                    drawPolyline(location)
                }
            }
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
            markers.add(createMarker(location[location.lastIndex]))
            markers.add(createTaxiMarker(location[0]))
            infoWindow.adapter = rootView?.let {
                MarkerInfoAdapter(requireContext(),
                    it, "거리", "비용")
            }!!
            infoWindow.open(markers[markers.lastIndex])

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

    private fun createMarker(location: Location): Marker {
        return Marker().apply {
            position = LatLng(location.lati.toDouble(), location.long.toDouble())    // 마커 좌표
            icon = OverlayImage.fromResource(R.drawable.ic_marker)
            iconTintColor = requireActivity().getColor(R.color.primaryColor)// 마커 색깔
            width = requireContext().getPxFromDp(40f)   // 마커 가로 크기
            height = requireContext().getPxFromDp(40f)  // 마커 세로 크기
            zIndex = 0  // 마커 높이
            onClickListener = Overlay.OnClickListener {     // 마커 클릭 리스너
                return@OnClickListener true
            }
            isHideCollidedMarkers = true    // 겹치면 다른 마커 숨기기
            map = naverMap  // 지도에 마커 표시
        }
    }

    private fun createTaxiMarker(location: Location): Marker {
        return Marker().apply {
            position = LatLng(location.lati.toDouble(), location.long.toDouble())    // 마커 좌표
            icon = OverlayImage.fromResource(R.drawable.ic_local_taxi)
            iconTintColor = requireActivity().getColor(R.color.greenTextColor)// 마커 색깔
            width = requireContext().getPxFromDp(40f)   // 마커 가로 크기
            height = requireContext().getPxFromDp(40f)  // 마커 세로 크기
            zIndex = 0  // 마커 높이
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

    }

    private fun initNaverMap() {
        val _naverMap =
            childFragmentManager.findFragmentById(R.id.fragmentContainer_location_tracking_taxi) as MapFragment?
                ?: MapFragment.newInstance().also {
                    childFragmentManager.beginTransaction()
                        .add(R.id.fragmentContainer_location_tracking_taxi, it)
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

            this@LocationTrackingTaxiFragment.uiSettings = this.uiSettings.apply {
                isCompassEnabled = false // 나침반 비활성화
                isZoomControlEnabled = false // 확대 축소 버튼 비활성화
                isScaleBarEnabled = false // 스케일 바 비활성화
                isLocationButtonEnabled = false // 기본 내 위치 버튼 비활성화
            }
        }
    }

    private fun getRoute() {
        callTaxiViewModel.addRouteSetting(
            RouteSetting(destination = Location(
                lati = ApplicationClass.prefs.startLatitude.toString(), long = ApplicationClass.prefs.startLongitude.toString()),
                startingPoint = Location(lati = ApplicationClass.prefs.latitude.toString(), long = ApplicationClass.prefs.longitude.toString()),
                checkState = true
            )
        )
    }

//    fun getMarkerBitmapFromView(drawable: Drawable?, context: Context): Bitmap? {
//        val customMarkerView: View =
//            (context.getSystemService(Context.LAYOUT_INFLATER_SERVICE) as LayoutInflater).inflate(
//                com.example.taxi.R.layout.item_taxi_marker,
//                null
//            )
//        val markerImage =
//            customMarkerView.findViewById<View>(R.id.user_marker_icon) as CircleImageView
//        customMarkerView.background =
//            context.resources.getDrawable(R.drawable.circle_profile)
//        markerImage.setImageDrawable(drawable)
//        customMarkerView.measure(View.MeasureSpec.UNSPECIFIED, View.MeasureSpec.UNSPECIFIED)
//        customMarkerView.layout(
//            0,
//            0,
//            customMarkerView.measuredWidth,
//            customMarkerView.measuredHeight
//        )
//        customMarkerView.buildDrawingCache()
//        val returnedBitmap = Bitmap.createBitmap(
//            customMarkerView.measuredWidth, customMarkerView.measuredHeight,
//            Bitmap.Config.ARGB_8888
//        )
//        val canvas = Canvas(returnedBitmap)
//        customMarkerView.draw(canvas)
//        return returnedBitmap
//    }


}